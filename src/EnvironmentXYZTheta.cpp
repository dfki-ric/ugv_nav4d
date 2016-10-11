#include "EnvironmentXYZTheta.hpp"
#include <sbpl/planners/planner.h>
#include <sbpl/utils/mdpconfig.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <base/Pose.hpp>
#include <fstream>
#include <dwa/SubTrajectory.hpp>
#include <backward/backward.hpp>


//If this is defined, additional debug data for visualization will be generated.
//It might slow down the planning

#define ENVIRONMENT_XYZ_THETA_GENERATE_DEBUG_DATA

backward::SignalHandling crashHandler;

using namespace std;
using namespace motion_planning_libraries;

namespace ugv_nav4d
{

const double costScaleFactor = 1000;


#define oassert(val) \
    if(!(val)) \
    {\
        std::cout << #val << std::endl; \
        std::cout << __FILE__ << ": " << __LINE__ << std::endl; \
        throw std::runtime_error("meeeeh"); \
    }



EnvironmentXYZTheta::EnvironmentXYZTheta(boost::shared_ptr<MLGrid> mlsGrid,
                                         const TraversabilityConfig& travConf,
                                         const SplinePrimitivesConfig& primitiveConfig,
                                         const Mobility& mobilityConfig) :
    travGen(travConf)
    , mlsGrid(mlsGrid)
    , availableMotions(primitiveConfig, mobilityConfig)
    , startThetaNode(nullptr)
    , startXYZNode(nullptr)
    , goalThetaNode(nullptr)
    , goalXYZNode(nullptr)
    , travConf(travConf)
    , mobilityConfig(mobilityConfig)
{
    numAngles = primitiveConfig.numAngles;
    travGen = TraversabilityGenerator3d(travConf);
    travGen.setMLSGrid(mlsGrid);
    searchGrid.setResolution(Eigen::Vector2d(travConf.gridResolution, travConf.gridResolution));
    searchGrid.extend(travGen.getTraversabilityMap().getNumCells());
    // FIXME z2 is divided by 2.0 to avoid intersecting the floor
    robotHalfSize << travConf.robotSizeX / 2, travConf.robotSizeY / 2, travConf.robotHeight/2/2;
}

void EnvironmentXYZTheta::clear()
{
    //clear the search grid
    for(maps::grid::LevelList<XYZNode *> &l : searchGrid)
    {
        for(XYZNode *n : l)
        {
            for(auto &tn : n->getUserData().thetaToNodes)
            {
                delete tn.second;
            }
            delete n;
        }
        l.clear();
    }
    
    idToHash.clear();

    startThetaNode = nullptr;
    startXYZNode = nullptr;
    
    goalThetaNode = nullptr;
    goalXYZNode = nullptr;
    
    for(int *p: StateID2IndexMapping)
    {
        delete p;
    }
    StateID2IndexMapping.clear();
}



EnvironmentXYZTheta::~EnvironmentXYZTheta()
{
    clear();
}

void EnvironmentXYZTheta::updateMap(boost::shared_ptr< maps::grid::MultiLevelGridMap< maps::grid::SurfacePatchBase > > mlsGrid)
{
    if(this->mlsGrid && this->mlsGrid->getResolution() != mlsGrid->getResolution())
        throw std::runtime_error("EnvironmentXYZTheta::updateMap : Error got MLSMap with different resolution");
    
    travGen.setMLSGrid(mlsGrid);
    this->mlsGrid = mlsGrid;

    clear();
}

EnvironmentXYZTheta::XYZNode* EnvironmentXYZTheta::createNewXYZState(TraversabilityGenerator3d::Node* travNode)
{
    XYZNode *xyzNode = new XYZNode(travNode->getHeight(), travNode->getIndex());
    xyzNode->getUserData().travNode = travNode;
    searchGrid.at(travNode->getIndex()).insert(xyzNode);

    return xyzNode;
}


EnvironmentXYZTheta::ThetaNode* EnvironmentXYZTheta::createNewStateFromPose(const Eigen::Vector3d& pos, double theta, XYZNode **xyzBackNode)
{
    TraversabilityGenerator3d::Node *travNode = travGen.generateStartNode(pos);
    if(!travNode)
    {
        cout << "createNewStateFromPose: Error Pose " << pos.transpose() << " is out of grid" << endl;
        throw runtime_error("Pose is out of grid");
    }
    
    //must be done, to correct height of start node
    if(!travNode->isExpanded() && !travGen.expandNode(travNode))
    {
        cout << "createNewStateFromPose: Error Pose " << pos.transpose() << " is not traversable" << endl;
        throw runtime_error("Pose is not traversable");
    }
    
    XYZNode *xyzNode = createNewXYZState(travNode);
    
    DiscreteTheta thetaD(theta, numAngles);
    
    if(xyzBackNode)
        *xyzBackNode = xyzNode;
    
    return createNewState(thetaD, xyzNode);
}


void EnvironmentXYZTheta::setGoal(const Eigen::Vector3d& goalPos, double theta)
{
    if(!startXYZNode)
        throw std::runtime_error("Error, start needs to be set before goal");
    
    goalThetaNode = createNewStateFromPose(goalPos, theta, &goalXYZNode);
    
    goalXYZNode->getUserData().travNode->setNotExpanded();
    
    travGen.expandAll(startXYZNode->getUserData().travNode);
    std::cout << "All expanded " << std::endl;
    precomputeCost();
    std::cout << "Heuristic computed" << std::endl;
}

void EnvironmentXYZTheta::setStart(const Eigen::Vector3d& startPos, double theta)
{
    startThetaNode = createNewStateFromPose(startPos, theta, &startXYZNode);
    startXYZNode->getUserData().travNode->setNotExpanded();
}

void EnvironmentXYZTheta::SetAllPreds(CMDPSTATE* state)
{
    //implement this if the planner needs access to predecessors

    SBPL_ERROR("ERROR in EnvNAV2D... function: SetAllPreds is undefined\n");
    throw EnvironmentXYZThetaException("SetAllPreds() not implemented");
}

void EnvironmentXYZTheta::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
    SBPL_ERROR("ERROR in EnvNAV2D... function: SetAllActionsandAllOutcomes is undefined\n");
    throw EnvironmentXYZThetaException("SetAllActionsandAllOutcomes() not implemented");
}


int EnvironmentXYZTheta::GetFromToHeuristic(int FromStateID, int ToStateID)
{
    throw std::runtime_error("GetFromToHeuristic not implemented");
//     const Hash &targetHash(idToHash[ToStateID]);
//     XYZNode *targetNode = targetHash.node;
// 
//     return GetHeuristic(FromStateID, targetHash.thetaNode, targetNode);
}

maps::grid::Vector3d EnvironmentXYZTheta::getStatePosition(const int stateID) const
{
    const Hash &sourceHash(idToHash[stateID]);
    const XYZNode *node = sourceHash.node;
    maps::grid::Vector3d ret;
    travGen.getTraversabilityMap().fromGrid(node->getIndex(), ret);
    ret.z() = node->getHeight();
    return ret;
}

const Motion& EnvironmentXYZTheta::getMotion(const int fromStateID, const int toStateID)
{
    int cost = -1;
    size_t motionId = 0;
    
    vector<int> successStates;
    vector<int> successStateCosts;
    vector<size_t> motionIds;
    
    GetSuccs(fromStateID, &successStates, &successStateCosts, motionIds);
    
    for(size_t i = 0; i < successStates.size(); i++)
    {
        if(successStates[i] == toStateID)
        {
            if(cost == -1 || cost > successStateCosts[i])
            {
                cost = successStateCosts[i];
                motionId = motionIds[i];
            }
        }
    }

    if(cost == -1)
        throw std::runtime_error("Internal Error: No matching motion for output path found");
    
    return availableMotions.getMotion(motionId);
}

const vector<PoseWithCell> &EnvironmentXYZTheta::getPoses(const int fromStateID, const int toStateID)
{
    const Motion& motion = getMotion(fromStateID, toStateID);
    return motion.intermediateSteps;
}


int EnvironmentXYZTheta::GetGoalHeuristic(int stateID)
{
    const Hash &sourceHash(idToHash[stateID]);
    const XYZNode *sourceNode = sourceHash.node;
    const TraversabilityGenerator3d::Node* travNode = sourceNode->getUserData().travNode;
    const ThetaNode *sourceThetaNode = sourceHash.thetaNode;
    
    const double sourceToGoalDist = travNodeIdToDistance[travNode->getUserData().id].distToGoal;
    const double timeTranslation = sourceToGoalDist / mobilityConfig.mSpeed;
    const double timeRotation = sourceThetaNode->theta.shortestDist(goalThetaNode->theta).getRadian() / mobilityConfig.mTurningSpeed;
    
    const int result = floor(std::max(timeTranslation, timeRotation) * costScaleFactor);
    oassert(result >= 0);
    return result;
}

int EnvironmentXYZTheta::GetStartHeuristic(int stateID)
{
    const Hash &targetHash(idToHash[stateID]);
    const XYZNode *targetNode = targetHash.node;
    const TraversabilityGenerator3d::Node* travNode = targetNode->getUserData().travNode;
    const ThetaNode *targetThetaNode = targetHash.thetaNode;

    const double startToTargetDist = travNodeIdToDistance[travNode->getUserData().id].distToStart;
    const double timeTranslation = startToTargetDist / mobilityConfig.mSpeed;
    double timeRotation = startThetaNode->theta.shortestDist(targetThetaNode->theta).getRadian() / mobilityConfig.mTurningSpeed;
    
    const int result = floor(std::max(timeTranslation, timeRotation) * costScaleFactor);
    oassert(result >= 0);
    return result;
}

bool EnvironmentXYZTheta::InitializeEnv(const char* sEnvFile)
{
    return true;
}

bool EnvironmentXYZTheta::InitializeMDPCfg(MDPConfig* MDPCfg)
{
    if(!goalThetaNode || !startThetaNode)
        return false;
    
    //initialize MDPCfg with the start and goal ids
    MDPCfg->goalstateid = goalThetaNode->id;
    MDPCfg->startstateid = startThetaNode->id;

    return true;
}

EnvironmentXYZTheta::ThetaNode *EnvironmentXYZTheta::createNewState(const DiscreteTheta &curTheta, XYZNode *curNode)
{
    ThetaNode *newNode = new ThetaNode(curTheta);
    newNode->id = idToHash.size();
    Hash hash(curNode, newNode);
    idToHash.push_back(hash);
    curNode->getUserData().thetaToNodes.insert(make_pair(curTheta, newNode));
    
    //this structure need to be extended for every new state that is added. 
    //Is seems it is later on filled in by the planner.
    
    //insert into and initialize the mappings
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for (int i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
        StateID2IndexMapping[newNode->id][i] = -1;
    }
    
    return newNode;
}

TraversabilityGenerator3d::Node *EnvironmentXYZTheta::movementPossible(TraversabilityGenerator3d::Node *fromTravNode, const maps::grid::Index &fromIdx, const maps::grid::Index &toIdx)
{
    if(toIdx == fromIdx)
        return fromTravNode;
    
    //get trav node associated with the next index
    TraversabilityGenerator3d::Node *targetNode = fromTravNode->getConnectedNode(toIdx);
    if(!targetNode)
    {
        throw std::runtime_error("should not happen");
    }
    
    if(!targetNode->isExpanded())
    {
        //current node is not drivable
        if(!travGen.expandNode(targetNode))
        {
            return nullptr;
        }
    }
    
    if(targetNode->getType() != maps::grid::TraversabilityNodeBase::TRAVERSABLE)
    {
        return nullptr;
    }
    
    //TODO add additionalCosts if something is near this node etc
    
    return targetNode;
}

void EnvironmentXYZTheta::GetSuccs(int SourceStateID, vector< int >* SuccIDV, vector< int >* CostV)
{
    std::vector<size_t> motionId;
    GetSuccs(SourceStateID, SuccIDV, CostV, motionId);
}


void EnvironmentXYZTheta::GetSuccs(int SourceStateID, vector< int >* SuccIDV, vector< int >* CostV, vector< size_t >& motionIdV)
{
    SuccIDV->clear();
    CostV->clear();
    motionIdV.clear();
    const Hash &sourceHash(idToHash[SourceStateID]);
    XYZNode *sourceNode = sourceHash.node;
    
#ifdef ENVIRONMENT_XYZ_THETA_GENERATE_DEBUG_DATA    
    Eigen::Vector3d succ((sourceNode->getUserData().travNode->getIndex().x() + 0.5) * travConf.gridResolution,
                         (sourceNode->getUserData().travNode->getIndex().y() + 0.5) * travConf.gridResolution,
                        sourceNode->getUserData().travNode->getHeight());
    debugSuccessors.push_back(this->mlsGrid->getLocalFrame().inverse(Eigen::Isometry) * succ);
#endif
    
    ThetaNode *thetaNode = sourceHash.thetaNode;
    maps::grid::Index sourceIndex = sourceNode->getIndex();
    XYZNode *curNode = sourceNode;

    TraversabilityGenerator3d::Node *travNode = curNode->getUserData().travNode;
    if(!travNode->isExpanded())
    {
        //current node is not drivable
        if(!travGen.expandNode(travNode))
        {
            return;
        }
    }

    for(const Motion &motion : availableMotions.getMotionForStartTheta(thetaNode->theta))
    {
        travNode = curNode->getUserData().travNode;
        maps::grid::Index curIndex = curNode->getIndex();
        std::vector<TraversabilityGenerator3d::Node*> nodesOnPath;
        
        for(const PoseWithCell &diff : motion.intermediateSteps)
        {
            maps::grid::Index newIndex = curIndex;
            
            //diff is always a full offset to the start position
            newIndex = sourceIndex + diff.cell;

            travNode = movementPossible(travNode, curIndex, newIndex);
            nodesOnPath.push_back(travNode);
            
            if(!travNode)
            {
                break;
            }
            curIndex = newIndex;
        }
        
        if(!travNode)
        {
            continue;
        }
        
        maps::grid::Index finalPos(sourceIndex);
        finalPos.x() += motion.xDiff;
        finalPos.y() += motion.yDiff;
        
        travNode = movementPossible(travNode, curIndex, finalPos);
        nodesOnPath.push_back(travNode);
        if(!travNode)
            continue;
        
        if(!checkCollisions(nodesOnPath, motion))
        {
            continue;
        }
        
        curIndex = finalPos;
        
        //goal from source to the end of the motion was valid
        XYZNode *successXYNode = nullptr;
        ThetaNode *successthetaNode = nullptr;
        
        //check if we can connect to the existing graph
        const auto &candidateMap = searchGrid.at(curIndex);

        if(travNode->getIndex() != curIndex)
            throw std::runtime_error("Internal error, indexes to not match");
        
        XYZNode searchTmp(travNode->getHeight(), travNode->getIndex());
        
        //note, this works, as the equals check is on the height, not the node itself
        auto it = candidateMap.find(&searchTmp);

        if(it != candidateMap.end())
        {
            //found a node with a matching height
            successXYNode = *it;
        }
        else
        {
            successXYNode = createNewXYZState(travNode);
        }

        const auto &thetaMap(successXYNode->getUserData().thetaToNodes);
        
        auto thetaCandidate = thetaMap.find(motion.endTheta);
        if(thetaCandidate != thetaMap.end())
        {
            successthetaNode = thetaCandidate->second;
        }
        else
        {
            successthetaNode = createNewState(motion.endTheta, successXYNode);
        }
               
        const double avgSlope = getAvgSlope(nodesOnPath) * travConf.slopeMetricScale;
        
        SuccIDV->push_back(successthetaNode->id);
        
         oassert(int(motion.baseCost + motion.baseCost * avgSlope) >= motion.baseCost);
        oassert(motion.baseCost > 0);
        CostV->push_back(int(motion.baseCost + motion.baseCost * avgSlope));
        motionIdV.push_back(motion.id);
    }
}

bool EnvironmentXYZTheta::checkCollisions(const std::vector< TraversabilityGenerator3d::Node* >& path,
                                          const Motion& motion) const
{
    //the final pose is part of the path but not of the poses.
    //Thus the size should always differ by one.
    oassert(motion.intermediateSteps.size() + 1 == path.size());
    
    const double robotHeight = travConf.robotHeight;

    for(size_t i = 0; i < path.size(); ++i)
    {
        const TraversabilityGenerator3d::Node* node(path[i]);
        //path contains the final element while intermediatePoses does not.
        const double zRot = i < motion.intermediateSteps.size() ?
                            motion.intermediateSteps[i].pose.orientation :
                            motion.endTheta.getRadian();

        // calculate robot position in local grid coordinates
        // TODO make this a member method of GridMap
        maps::grid::Vector3d robotPosition;
        robotPosition <<
                (node->getIndex().cast<double>() + Eigen::Vector2d(0.5, 0.5)).cwiseProduct(travGen.getTraversabilityMap().getResolution()),
                node->getHeight() + robotHeight * 0.5;
        
        const Eigen::Vector3d planeNormal = node->getUserData().plane.normal();
        oassert(planeNormal.allFinite()); 

        //FIXME names
        const Eigen::Quaterniond zRotAA( Eigen::AngleAxisd(zRot, Eigen::Vector3d::UnitZ()) ); // TODO these could be precalculated
        const Eigen::Quaterniond rotAA = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), planeNormal);
        const Eigen::Quaterniond rotQ = rotAA * zRotAA;

        // further calculations are more efficient with rotation matrix:
        const Eigen::Matrix3d rot = rotQ.toRotationMatrix();
        
        //find min/max for bounding box
        const Eigen::Vector3d extends = rot.cwiseAbs() * robotHalfSize;
        const Eigen::Vector3d min = robotPosition - extends;
        const Eigen::Vector3d max = robotPosition + extends;
        
        const Eigen::AlignedBox3d aabb(min, max); //aabb around the rotated robot bounding box    

        
        const Eigen::Matrix3d rotInv = rot.transpose();
        bool intersects = false;
        mlsGrid->intersectAABB_callback(aabb,
            [&rotInv, &intersects, this, &robotPosition, &rotQ, &rot]
            (const maps::grid::Index& idx, const maps::grid::SurfacePatchBase& p)
            {
                //FIXME this actually only tests if the top of the patch intersects with the robot
                maps::grid::Vector3d pos;
                double z = p.getMax();
                pos << (idx.cast<double>() + Eigen::Vector2d(0.5, 0.5)).cwiseProduct(this->mlsGrid->getResolution()), z;
                //transform pos into coordinate system of oriented bounding box
                pos -= robotPosition;
                pos = rotInv * pos;
                
                if((abs(pos.array()) <= this->robotHalfSize.array()).all())
                {
                    //found at least one patch that is inside the oriented boundingbox
#ifdef ENVIRONMENT_XYZ_THETA_GENERATE_DEBUG_DATA
                    this->debugCollisionPoses.push_back(base::Pose(this->mlsGrid->getLocalFrame().inverse(Eigen::Isometry) * robotPosition, rotQ));
#endif
                    intersects = true;
                    return true;//abort intersection check
                }
                return false; //continue intersection check
            });
        
        //abort if one of the path elements intersects the map
        if(intersects)
            return false;
    }
    
    return true;
}

Eigen::AlignedBox3d EnvironmentXYZTheta::getRobotBoundingBox() const
{
    //FIXME implement
    const Eigen::Vector3d min(0, 0, 0);
    const Eigen::Vector3d max(0.5, 1.0, 0.2);
    return Eigen::AlignedBox3d(min, max);
}

void EnvironmentXYZTheta::GetPreds(int TargetStateID, vector< int >* PredIDV, vector< int >* CostV)
{
    SBPL_ERROR("ERROR in EnvNAV2D... function: GetPreds is undefined\n");
    throw EnvironmentXYZThetaException("GetPreds() not implemented");
}

int EnvironmentXYZTheta::SizeofCreatedEnv()
{
    return static_cast<int>(idToHash.size());
}

void EnvironmentXYZTheta::PrintEnv_Config(FILE* fOut)
{
    throw EnvironmentXYZThetaException("PrintEnv_Config() not implemented");
}

void EnvironmentXYZTheta::PrintState(int stateID, bool bVerbose, FILE* fOut)
{
    const Hash &hash(idToHash[stateID]);
    
    std::stringbuf buffer;
    std::ostream os (&buffer);
    os << "State "<< stateID << " coordinate " << hash.node->getIndex().transpose() << " " << hash.node->getHeight() << " Theta " << hash.thetaNode->theta << endl;
    
    if(fOut)
        fprintf(fOut, "%s", buffer.str().c_str());
    else
        std::cout <<  buffer.str();
    
}

vector<Motion> EnvironmentXYZTheta::getMotions(const vector< int >& stateIDPath)
{
    vector<Motion> result;
    if(stateIDPath.size() >= 2)
    {
        for(size_t i = 0; i < stateIDPath.size() -1; ++i)
        {
            result.push_back(getMotion(stateIDPath[i], stateIDPath[i + 1]));
        }
    }
    return result;
}


void EnvironmentXYZTheta::getTrajectory(const vector< int >& stateIDPath, vector< base::Trajectory >& result)
{
    if(stateIDPath.size() < 2)
        return;
    
    result.clear();
    
    size_t lastMotion = getMotion(stateIDPath[0], stateIDPath[1]).id;
    
    base::Trajectory curPart;
    
    std::vector<base::Vector3d> positions;

    for(size_t i = 0; i < stateIDPath.size() - 1; ++i)
    {
        const Motion &curMotion(getMotion(stateIDPath[i], stateIDPath[i+1]));
        
        if(lastMotion != curMotion.id)
        {
            curPart.spline.interpolate(positions);
            curPart.speed = curMotion.speed;
            positions.clear();
            result.push_back(curPart);
        }

        const maps::grid::Vector3d start = getStatePosition(stateIDPath[i]);
        const Hash &startHash(idToHash[stateIDPath[i]]);
        const maps::grid::Index startIndex(startHash.node->getIndex());
        maps::grid::Index lastIndex = startIndex;
        TraversabilityGenerator3d::Node *curNode = startHash.node->getUserData().travNode;
        
        for(const PoseWithCell &pwc : curMotion.intermediateSteps)
        {
            base::Vector3d pos(pwc.pose.position.x() + start.x(), pwc.pose.position.y() + start.y(), start.z());
            maps::grid::Index curIndex = startIndex + pwc.cell;

            if(curIndex != lastIndex)
            {
                TraversabilityGenerator3d::Node *nextNode = curNode->getConnectedNode(curIndex);
                if(!nextNode)
                {
                    for(auto *n : curNode->getConnections())
                        std::cout << "Con Node " << n->getIndex().transpose() << std::endl;;
                    throw std::runtime_error("Internal error, trajectory is not continous on tr grid");
                }
                
                curNode = nextNode;

                lastIndex = curIndex;
            }
            
            pos.z() = curNode->getHeight();
            
            if(positions.empty() || !(positions.back().isApprox(pos)))
            {
                //need to offset by start because the poses are relative to (0/0)
                positions.emplace_back(pos);
            }
            
        }
    }
    std::cout << stateIDPath.back() << " ";
    std::cout << std::endl;

    curPart.spline.interpolate(positions);
    curPart.speed = 0;
    result.push_back(curPart);
}

maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase* > EnvironmentXYZTheta::getTraversabilityBaseMap() const
{
    return travGen.getTraversabilityBaseMap();
}

const maps::grid::TraversabilityMap3d< TraversabilityGenerator3d::Node* >& EnvironmentXYZTheta::getTraversabilityMap() const
{
    return travGen.getTraversabilityMap();
}

const maps::grid::MultiLevelGridMap< maps::grid::SurfacePatchBase >& EnvironmentXYZTheta::getMlsMap() const
{
    return *(mlsGrid.get());
}

const PreComputedMotions& EnvironmentXYZTheta::getAvailableMotions() const
{
    return availableMotions;
}

double EnvironmentXYZTheta::getAvgSlope(std::vector<TraversabilityGenerator3d::Node*> path) const
{
    double slopeSum = 0;
    for(TraversabilityGenerator3d::Node* node : path)
    {
        slopeSum += node->getUserData().slope; 
    }
    return slopeSum / path.size();
}

double EnvironmentXYZTheta::getAnglebetweenPlaneAndXY(const Eigen::Hyperplane< double, int(3) >& plane) const
{
    const Eigen::Vector3d zNormal(Eigen::Vector3d::UnitZ());
    Eigen::Vector3d planeNormal = plane.normal();
    planeNormal.normalize(); //just in case
    const double angle = acos(planeNormal.dot(zNormal));
    oassert(!std::isnan(angle));
    oassert(!std::isinf(angle));
    return angle;
}

void EnvironmentXYZTheta::precomputeCost()
{
    std::vector<double> costToStart;
    std::vector<double> costToEnd;
    
    //FIXME test if using double max causes problems, if not, use it.
    const double maxDist = 99999; //big enough to never occur in reality. Small enough to not cause overflows when used by accident.
    
    dijkstraComputeCost(startXYZNode->getUserData().travNode, costToStart, maxDist);
    dijkstraComputeCost(goalXYZNode->getUserData().travNode, costToEnd, maxDist);
    
    assert(costToStart.size() == costToEnd.size());
    
    
    travNodeIdToDistance.resize(costToStart.size(), Distance(std::numeric_limits<double>::max(), std::numeric_limits<double>::max()));
    for(size_t i = 0; i < costToStart.size(); ++i)
    {
        travNodeIdToDistance[i].distToStart =  costToStart[i];
        travNodeIdToDistance[i].distToGoal =  costToEnd[i];
                
        if(i != startXYZNode->getUserData().travNode->getUserData().id &&
           i != goalXYZNode->getUserData().travNode->getUserData().id &&
           costToStart[i] <= 0)
        {
            throw std::runtime_error("Heuristic of node other than start or goal is 0");
        }
    }
}

//Adapted from: https://rosettacode.org/wiki/Dijkstra%27s_algorithm#C.2B.2B
void EnvironmentXYZTheta::dijkstraComputeCost(TraversabilityGenerator3d::Node* source,
                          std::vector<double> &outDistances, const double maxDist)
{
    using namespace maps::grid;
    
    outDistances.clear();
    outDistances.resize(travGen.getNumNodes(), maxDist);
    
    const int sourceId = source->getUserData().id;
    outDistances[sourceId] = 0;
    
    std::set<std::pair<double, TraversabilityGenerator3d::Node*>> vertexQ;
    vertexQ.insert(std::make_pair(outDistances[sourceId], source));
    
#ifdef ENVIRONMENT_XYZ_THETA_GENERATE_DEBUG_DATA
    std::set<std::pair<double, TraversabilityGenerator3d::Node*>> vertex_queue_debug;
    vertex_queue_debug.insert(std::make_pair(outDistances[sourceId], source));
#endif
    
    while (!vertexQ.empty()) 
    {
        double dist = vertexQ.begin()->first;
        TraversabilityGenerator3d::Node* u = vertexQ.begin()->second;
        vertexQ.erase(vertexQ.begin());
        
        const Eigen::Vector3d uPos(u->getIndex().x() * travConf.gridResolution,
                                   u->getIndex().y() * travConf.gridResolution,
                                   0);
        
        // Visit each edge exiting u
        for(TraversabilityNodeBase *v : u->getConnections())
        {   
            //skip all non traversable nodes. They will retain the maximum cost.
            if(v->getType() != TraversabilityNodeBase::TRAVERSABLE)
                continue;
            
            TraversabilityGenerator3d::Node * vCasted = static_cast<TraversabilityGenerator3d::Node*>(v);
            //FIXME use vector2d
            const Eigen::Vector3d vPos(vCasted->getIndex().x() * travConf.gridResolution,
                                       vCasted->getIndex().y() * travConf.gridResolution,
                                       0);

            const double distance = (vPos - uPos).norm();
            double distance_through_u = dist + distance;
            const int vId = vCasted->getUserData().id;
            
            if (distance_through_u < outDistances[vId])
            {
                vertexQ.erase(std::make_pair(outDistances[vId], vCasted));
#ifdef ENVIRONMENT_XYZ_THETA_GENERATE_DEBUG_DATA
                vertex_queue_debug.erase(std::make_pair(outDistances[vId], vCasted));
#endif
                outDistances[vId] = distance_through_u;
                vertexQ.insert(std::make_pair(outDistances[vId], vCasted));
#ifdef ENVIRONMENT_XYZ_THETA_GENERATE_DEBUG_DATA
                vertex_queue_debug.insert(std::make_pair(outDistances[vId], vCasted));
#endif
            }
        }
    }
    
#ifdef ENVIRONMENT_XYZ_THETA_GENERATE_DEBUG_DATA
    debugHeuristic.clear();
    for(const std::pair<double, TraversabilityGenerator3d::Node*>& it : vertex_queue_debug)
    {
        const int cost = it.first;
        const TraversabilityGenerator3d::Node* node = it.second;
        Eigen::Vector3d pos(node->getIndex().x() * travConf.gridResolution, node->getIndex().y() * travConf.gridResolution, node->getHeight());
        pos = travGen.getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
        debugHeuristic.push_back(Eigen::Vector4d(pos.x(), pos.y(), pos.z(), cost));
    }
#endif
}

}
