#include "EnvironmentXYZTheta.hpp"
#include <sbpl/planners/planner.h>
#include <sbpl/utils/mdpconfig.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <base/Pose.hpp>
#include <fstream>
#include <dwa/SubTrajectory.hpp>
#include <backward/backward.hpp>

backward::SignalHandling crashHandler;

using namespace std;
using namespace motion_planning_libraries;

namespace ugv_nav4d
{

const double costScaleFactor = 1000; //FIXME WTF?


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
    , robotModel(0.3, 0.1)    
    , availableMotions(primitiveConfig, robotModel, mobilityConfig)
    , startThetaNode(nullptr)
    , startXYZNode(nullptr)
    , goalThetaNode(nullptr)
    , goalXYZNode(nullptr)
    , travConf(travConf)
{
    numAngles = primitiveConfig.numAngles;
    travGen = TraversabilityGenerator3d(travConf);
    travGen.setMLSGrid(mlsGrid);
    searchGrid.setResolution(Eigen::Vector2d(travConf.gridResolution, travConf.gridResolution));
    searchGrid.extend(travGen.getTraversabilityMap().getNumCells());
    // FIXME get real value from somewhere
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
    //FIXME
//     goalXYZNode->getUserData().travNode->setDistToStart(std::numeric_limits<double>::max());
    
    travGen.expandAll(startXYZNode->getUserData().travNode);
    std::cout << "All expanded " << std::endl;
    precomputeCost();
    std::cout << "Heuristic computed" << std::endl;
}

void EnvironmentXYZTheta::setStart(const Eigen::Vector3d& startPos, double theta)
{
    startThetaNode = createNewStateFromPose(startPos, theta, &startXYZNode);
    
    //FIXME why was this here?
//     for(auto *n : startXYZNode->getUserData().travNode->getConnections())
//     {
//         n->setDistToStart(std::numeric_limits< double >::max());
//     }
    //FIXME
//     startXYZNode->getUserData().travNode->setDistToStart(0);
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

int EnvironmentXYZTheta::GetHeuristic(int stateID, EnvironmentXYZTheta::ThetaNode* targetThetaNode, EnvironmentXYZTheta::XYZNode* targetXYZNode) const
{
    const Hash &sourceHash(idToHash[stateID]);
    XYZNode *sourceNode = sourceHash.node;

    //FIXME distToStart
//     double dist = sourceNode->getUserData().travNode->getDistToStart() * travConf.gridResolution;
    const double dist = 1;
    double timeTranslation = dist / robotModel.translationalVelocity;
    
    double timeRotation = sourceHash.thetaNode->theta.shortestDist(targetThetaNode->theta).getRadian() / robotModel.rotationalVelocity;
    
//     std::cout << "Theta " << sourceHash.thetaNode->theta << " to " << targetThetaNode->theta << " shortest dist is " << sourceHash.thetaNode->theta.shortestDist(targetThetaNode->theta) << " rad " << sourceHash.thetaNode->theta.shortestDist(targetThetaNode->theta).getRadian() << std::endl;
//     
//     std::cout << "Heuristic " << stateID << " " << sourceNode->getIndex().transpose() << " to " <<  targetXYZNode->getIndex().transpose() << " timeTranslation " << timeTranslation << " timeRotation " << timeRotation << " result "<< floor(std::max(timeTranslation, timeRotation) * costScaleFactor) << std::endl;
    
    return floor(std::max(timeTranslation, timeRotation) * costScaleFactor);
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
    
    //FIXME distToStart
     const double sourceToGoalDist = travNodeIdToDistance[travNode->getUserData().id].distToGoal;
//     const double sourceToGoalDist = (travNode->getIndex() - goalXYZNode->getUserData().travNode->getIndex()).cast<double>().norm() * travConf.gridResolution;
    const double timeTranslation = sourceToGoalDist / robotModel.translationalVelocity;
    const double timeRotation = sourceThetaNode->theta.shortestDist(goalThetaNode->theta).getRadian() / robotModel.rotationalVelocity;
    
    const int result = floor(std::max(timeTranslation, timeRotation) * costScaleFactor);
//     std::cout << "GetGoalHeuristic: travnode id:" << travNode->getUserData().id << ", coord: " << sourceNode->getIndex().transpose() << ", result:"  << result << std::endl;
    return result;
}

int EnvironmentXYZTheta::GetStartHeuristic(int stateID)
{
    const Hash &targetHash(idToHash[stateID]);
    const XYZNode *targetNode = targetHash.node;
    const TraversabilityGenerator3d::Node* travNode = targetNode->getUserData().travNode;
    const ThetaNode *targetThetaNode = targetHash.thetaNode;
    
    //FIXME distToStart

    const double startToTargetDist = travNodeIdToDistance[travNode->getUserData().id].distToStart;
//     const double startToTargetDist = (travNode->getIndex() - startXYZNode->getUserData().travNode->getIndex()).cast<double>().norm() * travConf.gridResolution;
    
    const double timeTranslation = startToTargetDist / robotModel.translationalVelocity;
    double timeRotation = startThetaNode->theta.shortestDist(targetThetaNode->theta).getRadian() / robotModel.rotationalVelocity;
    
    const int result = floor(std::max(timeTranslation, timeRotation) * costScaleFactor);;
//     std::cout << "GetStartHeuristic: travnode id:" << travNode->getUserData().id << ", result:"  << result << std::endl;
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
        //FIXME this should not happen !
        std::cout << "THIS SHOULD NOT HAPPEN" << std::endl;
        return nullptr;
    }
    
    if(!targetNode->isExpanded())
    {
        //current node is not drivable
        if(!travGen.expandNode(targetNode))
        {
//             std::cout << "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB" << std::endl;
            return nullptr;
        }
    }
    
    if(targetNode->getType() != maps::grid::TraversabilityNodeBase::TRAVERSABLE)
    {
//         std::cout << "NODE NOT TRAVERSABLE: " << targetNode->getIndex().transpose() << std::endl;
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
    
    ThetaNode *thetaNode = sourceHash.thetaNode;
    
    maps::grid::Index sourceIndex = sourceNode->getIndex();
    
    XYZNode *curNode = sourceNode;

    TraversabilityGenerator3d::Node *travNode = curNode->getUserData().travNode;
    if(!travNode->isExpanded())
    {
        //current node is not drivable
        if(!travGen.expandNode(travNode))
        {
//             cout << "Node " << travNode->getIndex().transpose() << "Is not drivable" << endl;
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
//         std::cout << "COLLLIIIIISSSSSION" << std::endl;
//         continue;
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
        
//         const int startHeu = GetStartHeuristic(SourceStateID);
//                 
//         const int curHeu = GetStartHeuristic(successthetaNode->id);
//         
         const double avgSlope = getAvgSlope(nodesOnPath) * travConf.slopeMetricScale;
//         
//         if(std::abs(startHeu - curHeu) > int(motion.baseCost + motion.baseCost * avgSlope))
//         {
//             std::cout <<  "sourceNode->getIndex(): " << sourceNode->getIndex().transpose() << std::endl;
//             std::cout <<  "successthetaNode->getIndex(): " << successXYNode->getIndex() << std::endl;
//             std::cout << "norm: " << (sourceNode->getIndex() - successXYNode->getIndex()).norm() << std::endl;
//             
//             std::cout << "startHeu: " << startHeu << std::endl;
//             std::cout << "curHeu: " << curHeu << std::endl;
//             std::cout << "motion.baseCost: " << motion.baseCost << std::endl;
//             std::cout << "int(motion.baseCost + motion.baseCost * avgSlope): " << int(motion.baseCost + motion.baseCost * avgSlope) << std::endl;
//         }
        
//         oassert(std::abs(startHeu - curHeu) <= int(motion.baseCost + motion.baseCost * avgSlope));
        
        SuccIDV->push_back(successthetaNode->id);
        
        oassert(int(motion.baseCost + motion.baseCost * avgSlope) >= motion.baseCost);
        oassert(motion.baseCost > 0);
        CostV->push_back(int(motion.baseCost + motion.baseCost * avgSlope));
        motionIdV.push_back(motion.id);
    }
    
//     std::cout << "SUCCS: " << SourceStateID << " -> ";
//     for(int id : *SuccIDV)
//     {
//         std::cout << id << ", ";
//     }
//     std::cout << std::endl;
    
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

        const auto intersectingPatches = mlsGrid->intersectAABB(aabb);
        if(intersectingPatches.size() > 0)
        {
            //the collision is inside the aabb. Still need to check whether at least
            //one patch is inside the real box
            const Eigen::Matrix3d rotInv = rot.transpose();
            for(const pair<maps::grid::Index, const maps::grid::SurfacePatchBase*>& patch : intersectingPatches)
            {
                // FIXME this actually only tests if the top of the patch intersects with the robot
                maps::grid::Vector3d pos;
                double z = patch.second->getMax();
                pos << (patch.first.cast<double>() + Eigen::Vector2d(0.5, 0.5)).cwiseProduct(mlsGrid->getResolution()), z;
                //transform pos into coordinate system of oriented bounding box
                pos -= robotPosition;
                pos = rotInv * pos;
                
                if( (abs(pos.array()) <= robotHalfSize.array()).all())
                {
                    intersectionPositions.push_back(rot * pos + robotPosition);
                    debugCollisionPoses.push_back(base::Pose(mlsGrid->getLocalFrame().inverse(Eigen::Isometry) * robotPosition, rotQ));
                    //found at least one patch that is inside the oriented bchecounding box
                    return false;
                }
            }
        }
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

// const Eigen::AlignedBox3d& EnvironmentXYZTheta::PreComputedBoundingBoxes::getBoundingBox(const int theta) const
// {
//   //TODO
// }

// void EnvironmentXYZTheta::PreComputedBoundingBoxes::preComputeBoxes(const double robotXSize, const double robotYSize,
//                                                                     const double robotZSize, const int numAngles,
//                                                                     boost::shared_ptr<maps::grid::MultiLevelGridMap<maps::grid::SurfacePatchBase> > mlsGrid)
// {
//     const double x2 = robotXSize / 2.0;
//     const double y2 = robotYSize / 2.0;
//     const double z2 = robotZSize / 2.0;
// 
//     maps::grid::Vector3d robotPosition(0, 0, 0);
// 
//     //TODO improve performance by pre calculating lots of stuff
//     //create rotated robot bounding box
//     Eigen::Matrix<double, 3, 8> corners; //colwise corner vectors
//     corners.col(0) << -x2, -y2, -(z2/2.0); //FIXME replace -(z2/2.0) with something real.
//     corners.col(1) << x2, -y2, -(z2/2.0);
//     corners.col(2) << x2, y2, -(z2/2.0);
//     corners.col(3) << -x2, y2, -(z2/2.0);
//     corners.col(4) << x2, -y2, z2/2.0;
//     corners.col(5) << x2, y2, z2/2.0;
//     corners.col(6) << -x2, y2, z2/2.0;
//     corners.col(7) << -x2, -y2, z2/2.0;
//     
//     const double stepDist = 2.0 * M_PI / numAngles;
//     const double stepDist2 = stepDist / 2.0;
//     std::cout << "rotations: " << std::endl;
//     for(int i = 0; i < numAngles; ++i)
//     {
//         const double rot = i * stepDist + stepDist2;
//         std::cout << i << std::endl;
//         const Eigen::Matrix3d rot = Eigen::AngleAxisd(rot, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
//         const Eigen::Matrix<double, 3, 8> rotatedCorners = (rot * corners).colwise() + robotPosition;
// 
//         //find min/max for bounding box
//         const Eigen::Vector3d min = rotatedCorners.rowwise().minCoeff();
//         const Eigen::Vector3d max = rotatedCorners.rowwise().maxCoeff();
// 
//         const Eigen::AlignedBox3d aabb(min, max); //aabb around the rotated robot bounding box
//     }
// }

const PreComputedMotions& EnvironmentXYZTheta::getAvailableMotions() const
{
    return availableMotions;
}

double EnvironmentXYZTheta::getAvgSlope(std::vector<TraversabilityGenerator3d::Node*> path) const
{
    double slopeSum = 0;
    for(TraversabilityGenerator3d::Node* node : path)
    {
        slopeSum += getAnglebetweenPlaneAndXY(node->getUserData().plane); 
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
    
    bool val =(angle >= 0);
    
    if(!val)
    {
        std::cout << "znormal: " << zNormal.transpose() << std::endl;
        std::cout << "planeNormal: " << planeNormal.transpose() << std::endl;
        std::cout << "angle: " << angle << std::endl;
        std::cout << "planeNormal.dot(zNormal): " << planeNormal.dot(zNormal) << std::endl;
    }
    
    oassert(val);
    return angle;
}

void EnvironmentXYZTheta::precomputeCost()
{
    std::vector<double> costToStart;
    std::vector<double> costToEnd;
    dijkstraComputeCost(goalXYZNode->getUserData().travNode, costToEnd);
    dijkstraComputeCost(startXYZNode->getUserData().travNode, costToStart);
    
    assert(costToStart.size() == costToEnd.size());
    
    travNodeIdToDistance.resize(costToStart.size(), Distance(std::numeric_limits<double>::max(), std::numeric_limits<double>::max()));
    for(int i = 0; i < costToStart.size(); ++i)
    {
        travNodeIdToDistance[i].distToStart = costToStart[i];
        travNodeIdToDistance[i].distToGoal = costToEnd[i];
        
        if(i != startXYZNode->getUserData().travNode->getUserData().id &&
           i != goalXYZNode->getUserData().travNode->getUserData().id &&
           costToStart[i] <= 0)
        {
            std::cout << "heuristic: " << costToStart[i] << std::endl;
        }
        
    }
}

//Adapted from: https://rosettacode.org/wiki/Dijkstra%27s_algorithm#C.2B.2B
void EnvironmentXYZTheta::dijkstraComputeCost(TraversabilityGenerator3d::Node* source,
                          std::vector<double> &cost)
{
    using weight_t = double;
    using namespace maps::grid;
    
    int n = travGen.getNumNodes();
    cost.clear();
    cost.resize(n, std::numeric_limits< int >::max());
    const int sourceId = source->getUserData().id;
    cost[sourceId] = 0;
    std::set<std::pair<weight_t, TraversabilityGenerator3d::Node*>> vertex_queue;
    std::set<std::pair<weight_t, TraversabilityGenerator3d::Node*>> vertex_queue_debug; //FIXME remove after debug
    vertex_queue.insert(std::make_pair(cost[sourceId], source));
    vertex_queue_debug.insert(std::make_pair(cost[sourceId], source));
 
    while (!vertex_queue.empty()) 
    {
        weight_t dist = vertex_queue.begin()->first;
        TraversabilityGenerator3d::Node* u = vertex_queue.begin()->second;
        vertex_queue.erase(vertex_queue.begin());
        
        const Eigen::Vector3d uPos(u->getIndex().x() * travConf.gridResolution,
                                   u->getIndex().y() * travConf.gridResolution,
                                   u->getHeight());
        
        // Visit each edge exiting u
        for(TraversabilityNodeBase *v : u->getConnections())
        {   
            TraversabilityGenerator3d::Node * vCasted = static_cast<TraversabilityGenerator3d::Node*>(v);
            const Eigen::Vector3d vPos(vCasted->getIndex().x() * travConf.gridResolution,
                                       vCasted->getIndex().y() * travConf.gridResolution,
                                       vCasted->getHeight());
            //FIXME not sure if using height is a good idea
            const weight_t distance = (vPos - uPos).norm();
            weight_t distance_through_u = dist + distance;
            const int vId = vCasted->getUserData().id;
            if (distance_through_u < cost[vId])
            {
                vertex_queue.erase(std::make_pair(cost[vId], vCasted));
                vertex_queue_debug.erase(std::make_pair(cost[vId], vCasted));
                cost[vId] = distance_through_u;
                vertex_queue.insert(std::make_pair(cost[vId], vCasted));
                vertex_queue_debug.insert(std::make_pair(cost[vId], vCasted));
            }
        }
    }
    
    //FIXME remove after debug
    debugCost.clear();
    for(const std::pair<weight_t, TraversabilityGenerator3d::Node*>& it : vertex_queue_debug)
    {
        const int cost = it.first;
        const TraversabilityGenerator3d::Node* node = it.second;
        Eigen::Vector3d pos(node->getIndex().x() * travConf.gridResolution, node->getIndex().y() * travConf.gridResolution, node->getHeight());
        pos = travGen.getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
        debugCost.push_back(Eigen::Vector4d(pos.x(), pos.y(), pos.z(), cost));
    }
}

}
