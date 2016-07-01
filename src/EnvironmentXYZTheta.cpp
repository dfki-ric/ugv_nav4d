#include <motion_planning_libraries/sbpl/SbplMotionPrimitives.hpp>
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

EnvironmentXYZTheta::RobotModel::RobotModel(double tr, double rv) : translationalVelocity(tr), rotationalVelocity(rv)
{

}


void EnvironmentXYZTheta::PreComputedMotions::setMotionForTheta(const EnvironmentXYZTheta::Motion& motion, const DiscreteTheta& theta)
{
    if((int)thetaToMotion.size() <= theta.getTheta())
    {
        thetaToMotion.resize(theta.getTheta() + 1);
    }
    
    //check if a motion to this target destination already exist, if yes skip it.
    for(const Motion& m : thetaToMotion[theta.getTheta()])
    {
        if(m.xDiff == motion.xDiff && m.yDiff == motion.yDiff && m.endTheta == motion.endTheta)
        {
            std::cout << "WARNING: motion already exists (skipping): " << m.xDiff << ", " << m.yDiff << ", " << m.endTheta << std::endl;
            //TODO add check if intermediate poses are similar
            return;
        }
    }
    
    thetaToMotion[theta.getTheta()].push_back(motion);
}

void EnvironmentXYZTheta::PreComputedMotions::preComputeCost(EnvironmentXYZTheta::Motion& motion, const RobotModel &model)
{
    //compute linear and angular time
    double linear_distance = 0;
    Eigen::Vector2d lastPos(0, 0);
    for(const base::Pose2D &pos: motion.intermediatePoses)
    {
        linear_distance += (lastPos - pos.position).norm();
        lastPos = pos.position;
    }

    double linear_time = linear_distance / model.translationalVelocity;

    double angular_distance = motion.endTheta.shortestDist(motion.startTheta).getRadian();
    
    double angular_time = angular_distance / model.rotationalVelocity;
    
    motion.baseCost = ceil(std::max(angular_time, linear_time) * costScaleFactor * motion.costMultiplier);

//     std::cout << "Motion " << motion.xDiff << " " << motion.yDiff << " " << motion.endTheta << 
//     " lin dist " <<  linear_distance << " time " << linear_time << " angle diff " 
//     << angular_distance << " time " << angular_time << " cost " << motion.baseCost << " multiplier " << motion.costMultiplier <<  std::endl;
    

}


EnvironmentXYZTheta::EnvironmentXYZTheta(boost::shared_ptr< maps::grid::MultiLevelGridMap< maps::grid::SurfacePatchBase > > mlsGrid,
                                         const TraversabilityGenerator3d::Config &travConf,
                                         const motion_planning_libraries::SbplMotionPrimitives& primitives) : 
    travGen(travConf)
    , mlsGrid(mlsGrid)
    , robotModel(0.3, M_PI / 8)    
    , startThetaNode(nullptr)
    , startXYZNode(nullptr)
    , goalThetaNode(nullptr)
    , goalXYZNode(nullptr)
    , travConf(travConf)
{
    numAngles = 16;
    
    travGen.setMLSGrid(mlsGrid);
    
    searchGrid.setResolution(Eigen::Vector2d(travConf.gridResolution, travConf.gridResolution));
    searchGrid.extend(travGen.getTraversabilityMap().getNumCells());
    readMotionPrimitives(primitives);
}

EnvironmentXYZTheta::~EnvironmentXYZTheta()
{

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
    
//     std::cout << "Heuristic " << stateID << " " << sourceNode->getIndex().transpose() << " to " <<  targetXYZNode->getIndex().transpose() << " timeTranslation " << timeTranslation << " timeRotation " << timeRotation << " result "<< floor(std::max(timeTranslation, timeRotation) * costScaleFactor) << std::endl;
    
    return floor(std::max(timeTranslation, timeRotation) * costScaleFactor);
}

int EnvironmentXYZTheta::GetFromToHeuristic(int FromStateID, int ToStateID)
{
    const Hash &targetHash(idToHash[ToStateID]);
    XYZNode *targetNode = targetHash.node;

    return GetHeuristic(FromStateID, targetHash.thetaNode, targetNode);
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
    TraversabilityGenerator3d::Node *targetNode = nullptr;
    
    if(toIdx == fromIdx)
        return fromTravNode;
    
    //get trav node associated with the next index
    //TODO, this is slow, make it faster
    for(maps::grid::TraversabilityNodeBase *con: fromTravNode->getConnections())
    {
        if(toIdx == con->getIndex())
        {
            targetNode = static_cast<TraversabilityGenerator3d::Node *>(con);
            break;
        }
    }

    if(!targetNode)
    {
        cout << "No neighbour node for motion found " << endl;
        cout << "curIndex " << fromIdx.transpose() << " newIndex " << toIdx.transpose() << endl;
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
    
    //TODO add some collision testing
    
    //TODO add additionalCosts if something is near this node etc
    
    return targetNode;
}


void EnvironmentXYZTheta::GetSuccs(int SourceStateID, vector< int >* SuccIDV, vector< int >* CostV)
{
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
        
        int additionalCosts = 0;
        
        //FIXME intermediateCells has been removed (is not provided by config file)
        for(const maps::grid::Index &diff : motion.intermediateCells)
        {
            maps::grid::Index newIndex = curIndex;
            
            //diff is always a full offset to the start position
            newIndex = sourceIndex + diff;

            
            travNode = movementPossible(travNode, curIndex, newIndex);
            
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
        if(!travNode)
            continue;

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
//             cout << "Found existing XY node " << endl;
            //found a node with a matching height
            successXYNode = *it;
        }
        else
        {
//             std::cout << "Creating new XY Node " << curIndex.transpose() << std::endl;
            
            successXYNode = createNewXYZState(travNode);
        }

        const auto &thetaMap(successXYNode->getUserData().thetaToNodes);
        
//         for(const auto &e: thetaMap)
//         {
//             cout << "Theta Elem id " << e.second->id << " angle " << e.first << endl;
//         }
        
        auto thetaCandidate = thetaMap.find(motion.endTheta);
        if(thetaCandidate != thetaMap.end())
        {
//             cout << "Found existing State, reconnectiong graph " << endl;
            
            successthetaNode = thetaCandidate->second;
        }
        else
        {
            successthetaNode = createNewState(motion.endTheta, successXYNode);
        }
        
        
        SuccIDV->push_back(successthetaNode->id);
        CostV->push_back(motion.baseCost + additionalCosts);
    }
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
        fprintf(fOut, buffer.str().c_str());
    else
        std::cout <<  buffer.str();
    
}

void EnvironmentXYZTheta::readMotionPrimitives(const SbplMotionPrimitives& primitives)
{

    for(const Primitive& prim : primitives.mListPrimitives)
    {
        Motion motion(numAngles);

        motion.xDiff = prim.mEndPose[0];
        motion.yDiff = prim.mEndPose[1];
        motion.endTheta =  DiscreteTheta(static_cast<int>(prim.mEndPose[2]), numAngles);
        motion.startTheta = DiscreteTheta(prim.mStartAngle, numAngles);
        motion.costMultiplier = prim.mCostMultiplier;

        std::vector<base::Pose2D> poses;
        bool allPositionsSame = true;
        base::Vector2d firstPos = prim.mIntermediatePoses.front().topRows(2);
        for(const base::Vector3d& pose : prim.mIntermediatePoses)
        {
            poses.emplace_back(base::Position2D(pose[0], pose[1]), pose[2]);
            if(pose.topRows(2) != firstPos)
            {
                allPositionsSame = false;
            }
        }

        if(!allPositionsSame)
        {
            //fit a spline through the primitive's intermediate points and use that spline to
            //create intermediate points for the motion.
            //This is done to ensure that every cell is hit by one intermediate point.
            //if the positions are different, normal spline interpolation works
            SubTrajectory spline;
            spline.interpolate(poses);
            const double splineLength = spline.getDistToGoal(spline.getStartParam());
            //set stepDist so small that we are guaranteed to oversample
            const double stepDist = travConf.gridResolution - (travConf.gridResolution / 2.0);
            double currentDist = 0;
            double currentParam = spline.getStartParam();
            while(currentDist < splineLength)
            {
            currentDist += stepDist;
            currentParam = spline.advance(currentParam, stepDist);
            const base::Pose2D currentPose = spline.getIntermediatePointNormalized(currentParam);
            
            //convert pose to grid
            const base::Vector3d position(currentPose.position.x(), currentPose.position.y(), 0);
            maps::grid::Index diff;
            //only add pose if it is in a different cell than the one before
            if(!searchGrid.toGrid(position, diff, false))
            {
                throw EnvironmentXYZThetaException("Cannot convert intermediate Pose to grid cell");
            }
            
            if(motion.intermediateCells.size() == 0 || motion.intermediateCells.back() != diff)
            {
                motion.intermediateCells.push_back(diff);
                motion.intermediatePoses.push_back(currentPose);
//               std::cout << "intermediate poses: " << currentPose.position.transpose() << ", " << currentPose.orientation << 
//                            "[" << diff.transpose() << "]" << std::endl;
            }            
        }
    }
    else
    {
        //Since our index is only x/y based (i.e. it ignores rotation)
        //we do not need to add any intermediate positions for rotation-only
        //movements.
    }

    availableMotions.preComputeCost(motion, robotModel);

    availableMotions.setMotionForTheta(motion, motion.startTheta);

//     std::cout << "startTheta " <<  prim.mStartAngle << std::endl;
//     std::cout << "mEndPose " <<  prim.mEndPose.transpose() << std::endl;
// 
//     std::cout << "Adding Motion: 0, 0, " << motion.startTheta << " -> " << motion.xDiff << ", " << motion.yDiff << ", " << motion.endTheta << std::endl << std::endl;
    }
}



