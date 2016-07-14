#include "EnvironmentXYZTheta.hpp"
#include <sbpl/planners/planner.h>
#include <sbpl/utils/mdpconfig.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <base/Pose.hpp>
#include <fstream>
#include <dwa/SubTrajectory.hpp>

using namespace std;
using namespace motion_planning_libraries;

const double costScaleFactor = 1000;

EnvironmentXYZTheta::EnvironmentXYZTheta(boost::shared_ptr< maps::grid::MultiLevelGridMap< maps::grid::SurfacePatchBase > > mlsGrid,
                                         const TraversabilityGenerator3d::Config &travConf,
                                         const motion_planning_libraries::MotionPrimitivesConfig &primitiveConfig) : 
    travGen(travConf)
    , mlsGrid(mlsGrid)
    , robotModel(0.3, 0.1)    
    , availableMotions(primitiveConfig, robotModel)
    , startThetaNode(nullptr)
    , startXYZNode(nullptr)
    , goalThetaNode(nullptr)
    , goalXYZNode(nullptr)
    , travConf(travConf)
{
    numAngles = 16;
    travGen = TraversabilityGenerator3d(travConf);
    travGen.setMLSGrid(mlsGrid);
    searchGrid.setResolution(Eigen::Vector2d(travConf.gridResolution, travConf.gridResolution));
    std::cout  << "Extending map to " << travGen.getTraversabilityMap().getNumCells().transpose() << std::endl;
    searchGrid.extend(travGen.getTraversabilityMap().getNumCells());
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
        throw std::runtime_error("Error, start needs to be set before start");
    
    goalThetaNode = createNewStateFromPose(goalPos, theta, &goalXYZNode);
    
    goalXYZNode->getUserData().travNode->setNotExpanded();
    
    travGen.expandAll(goalXYZNode->getUserData().travNode);
    std::cout << "All expanded " << std::endl;
}

void EnvironmentXYZTheta::setStart(const Eigen::Vector3d& startPos, double theta)
{
    startThetaNode = createNewStateFromPose(startPos, theta, &startXYZNode);
    
    for(auto *n : startXYZNode->getUserData().travNode->getConnections())
    {
        n->setDistToStart(std::numeric_limits< double >::max());
    }
    
    startXYZNode->getUserData().travNode->setDistToStart(std::numeric_limits< double >::max());
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

    double dist = sourceNode->getUserData().travNode->getDistToStart();
    
    double timeTranslation = dist / robotModel.translationalVelocity;
    
    double timeRotation = sourceHash.thetaNode->theta.shortestDist(targetThetaNode->theta).getRadian() / robotModel.rotationalVelocity;
    
//     std::cout << "Theta " << sourceHash.thetaNode->theta << " to " << targetThetaNode->theta << " shortest dist is " << sourceHash.thetaNode->theta.shortestDist(targetThetaNode->theta) << " rad " << sourceHash.thetaNode->theta.shortestDist(targetThetaNode->theta).getRadian() << std::endl;
//     
//     std::cout << "Heuristic " << stateID << " " << sourceNode->getIndex().transpose() << " to " <<  targetXYZNode->getIndex().transpose() << " timeTranslation " << timeTranslation << " timeRotation " << timeRotation << " result "<< floor(std::max(timeTranslation, timeRotation) * costScaleFactor) << std::endl;
    
    return floor(std::max(timeTranslation, timeRotation) * costScaleFactor);
}

int EnvironmentXYZTheta::GetFromToHeuristic(int FromStateID, int ToStateID)
{
    const Hash &targetHash(idToHash[ToStateID]);
    XYZNode *targetNode = targetHash.node;

    return GetHeuristic(FromStateID, targetHash.thetaNode, targetNode);
}

maps::grid::Vector3d EnvironmentXYZTheta::getStatePosition(const int stateID) const
{
    const Hash &sourceHash(idToHash[stateID]);
    const XYZNode *node = sourceHash.node;
    maps::grid::Vector3d ret;
    mlsGrid->fromGrid(node->getIndex(), ret);
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

const vector<base::Pose2D> &EnvironmentXYZTheta::getPoses(const int fromStateID, const int toStateID)
{
    const Motion& motion = getMotion(fromStateID, toStateID);
    return motion.intermediatePoses;
}


int EnvironmentXYZTheta::GetGoalHeuristic(int stateID)
{
    return GetHeuristic(stateID, goalThetaNode, goalXYZNode);
}

int EnvironmentXYZTheta::GetStartHeuristic(int stateID)
{
//     std::cout << "Start " << std::endl;
    return GetHeuristic(stateID, startThetaNode, startXYZNode);
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
        return nullptr;
    }
    
    if(!targetNode->isExpanded())
    {
        //current node is not drivable
        if(!travGen.expandNode(targetNode))
        {
            return nullptr;
        }
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
//     std::cout << "GetSuccs " << SourceStateID << std::endl;
    
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
     
        int additionalCosts = 0;
        
        for(const maps::grid::Index &diff : motion.intermediateCells)
        {
            
            maps::grid::Index newIndex = curIndex;
            
            //diff is always a full offset to the start position
            newIndex = sourceIndex + diff;

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

        additionalCosts += (sourceIndex - finalPos).norm() + 1;
        
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
        
        SuccIDV->push_back(successthetaNode->id);
        CostV->push_back(motion.baseCost + additionalCosts);
        motionIdV.push_back(motion.id);
    }
}

bool EnvironmentXYZTheta::checkCollisions(const std::vector< TraversabilityGenerator3d::Node* >& path,
                                          const Motion& motion) const
{
    //the final pose is part of the path but not of the posese.
    //Thus the size should always differ by one.
    assert(motion.intermediatePoses.size() + 1 == path.size());
    
    std::vector<base::Pose2D> poses(motion.intermediatePoses);
    for(unsigned i = 0; i < path.size(); ++i)
    {
        const TraversabilityGenerator3d::Node* node(path[i]);
        const double robotSizeY = 0.8; //FIXME get real value frome somewhere
        const double robotSizeX = travConf.robotSizeX;
        const double robotHeight = travConf.robotHeight;
        const double x2 = robotSizeX / 2.0;
        const double y2 = robotSizeY / 2.0;
        const double z2 = robotHeight / 2.0;
        
        //path contains the final element while intermediatePoses does not.
        const double zRot = i < motion.intermediatePoses.size() ?
                            motion.intermediatePoses[i].orientation :
                            motion.endTheta.getRadian();
        
        maps::grid::Vector3d robotPosition;
        mlsGrid->fromGrid(node->getIndex(), robotPosition);
        robotPosition.z() = node->getHeight() + robotHeight / 2.0;
        
        //TODO improve performance by pre calculating lots of stuff
        //create rotated robot bounding box
        Eigen::Matrix<double, 3, 8> corners; //colwise corner vectors
        corners.col(0) << -x2, -y2, -(z2/2.0); //FIXME replace (z2/2.0) with real height?
        corners.col(1) << x2, -y2, -(z2/2.0);
        corners.col(2) << x2, y2, -(z2/2.0);
        corners.col(3) << -x2, y2, -(z2/2.0);
        corners.col(4) << x2, -y2, z2/2.0;
        corners.col(5) << x2, y2, z2/2.0;
        corners.col(6) << -x2, y2, z2/2.0;
        corners.col(7) << -x2, -y2, z2/2.0;

        
        const Eigen::Vector3d planeNormal = node->getUserData().plane.normal();       
        assert(planeNormal.allFinite()); 
        const Eigen::Vector3d boxNormal(0, 0, 1); //FIXME use UnitZ
        Eigen::Vector3d rotAxis = boxNormal.cross(planeNormal);
        rotAxis.normalize();
        const double rotAngle = acos(boxNormal.dot(planeNormal));
        
        //FIXME names
        const Eigen::Matrix3d zRotAA = Eigen::AngleAxisd(zRot, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
        const Eigen::Matrix3d rotAA = Eigen::AngleAxisd(rotAngle, rotAxis).toRotationMatrix();
        
        const Eigen::Matrix3d rot = rotAA * zRotAA;
        
        const Eigen::Matrix<double, 3, 8> rotatedCorners = (rot * corners).colwise() + robotPosition;
//         debugRotatedBoxes.push_back(rotatedCorners);
        
        //find min/max for bounding box
        const Eigen::Vector3d min = rotatedCorners.rowwise().minCoeff();
        const Eigen::Vector3d max = rotatedCorners.rowwise().maxCoeff();
        
         const Eigen::AlignedBox3d aabb(min, max); //aabb around the rotated robot bounding box
        //get all grid cells below robot bounding box
        maps::grid::Index minIdx;
        if(!mlsGrid->toGrid(Eigen::Vector3d(min.x(), min.y(), .0), minIdx, true))
        {
            std::cout << "bounding box outside map " << Eigen::Vector3d(min.x(), min.y(), .0).transpose() << std::endl;
            throw std::runtime_error("bounding box outside map");
        }
        
        maps::grid::Index maxIdx;
        if(!mlsGrid->toGrid(Eigen::Vector3d(max.x(), max.y(), .0), maxIdx, true))
        {
            std::cout << "bounding box outside map " << Eigen::Vector3d(min.x(), min.y(), .0).transpose() << std::endl;
            throw std::runtime_error("bounding box outside map");
        }       

        std::size_t numIntersections = 0;
        auto view = mlsGrid->intersectCuboid(aabb, numIntersections);
        if(numIntersections > 0)
        {
            debugCollisions.emplace_back(min, max);
            debugRotatedBoxes.push_back(rotatedCorners);
            return false;
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

void EnvironmentXYZTheta::getTrajectory(const vector< int >& solution, std::vector< base::Trajectory >& trajectory)
{
        if(solution.size() < 2)
        return;
    
    trajectory.clear();
    
    std::cout << "Solution: " << std::endl;
    size_t lastMotion = getMotion(solution[0], solution[1]).id;
    
    base::Trajectory curPart;
    
    std::vector<base::Vector3d> positions;
    
    
    
    for(size_t i = 0; i < solution.size() - 1; ++i)
    {
        const Motion &curMotion(getMotion(solution[i], solution[i+1]));
        
        std::cout << "Motion has id " << curMotion.id << std::endl;
        
        if(lastMotion != curMotion.id)
        {
            curPart.spline.interpolate(positions);
            curPart.speed = 0.2;
            positions.clear();
            trajectory.push_back(curPart);
        }

        std::cout << solution[i] << " ";
        const maps::grid::Vector3d start = getStatePosition(solution[i]);
        std::cout << "Intermediate Poses : " << curMotion.intermediatePoses.size() << std::endl;
        for(const base::Pose2D& pose : curMotion.intermediatePoses)
        {
            base::Vector3d pos(pose.position.x() + start.x(), pose.position.y() + start.y(), start.z());
            
//             updateHeight(pos);
            
            //need to offset by start because the poses are relative to (0/0)
            positions.emplace_back(pos);
            
            std::cout << "Intermediate position " << pos.transpose() << std::endl;
        }
    }
    std::cout << std::endl;

    curPart.spline.interpolate(positions);
    curPart.speed = 0;
    trajectory.push_back(curPart);
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


