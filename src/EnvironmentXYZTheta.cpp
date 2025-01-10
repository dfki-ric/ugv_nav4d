#include "EnvironmentXYZTheta.hpp"
#include <sbpl/planners/planner.h>
#include <sbpl/utils/mdpconfig.h>
#include <base/Pose.hpp>
#include <base/Spline.hpp>
#include <fstream>
#include "PathStatistic.hpp"
#include "Dijkstra.hpp"
#include <limits>
#include <base-logging/Logging.hpp>

#ifdef ENABLE_DEBUG_DRAWINGS
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>
#include <vizkit3d_debug_drawings/DebugDrawingColors.hpp>
#endif

using namespace std;
using namespace sbpl_spline_primitives;
using trajectory_follower::SubTrajectory;
using trajectory_follower::DriveMode;

namespace ugv_nav4d
{

#define oassert(val) \
    if(!(val)) \
    {\
        LOG_ERROR_S << #val; \
        LOG_ERROR_S << __FILE__ << ": " << __LINE__; \
        throw std::runtime_error("Error!"); \
    }

EnvironmentXYZTheta::EnvironmentXYZTheta(std::shared_ptr<const traversability_generator3d::TravMap3d> travMap,
                                         const traversability_generator3d::TraversabilityConfig& travConf,
                                         const SplinePrimitivesConfig& primitiveConfig,
                                         const Mobility& mobilityConfig) :
    travMap(travMap)
    , availableMotions(primitiveConfig, mobilityConfig)
    , startThetaNode(nullptr)
    , startXYZNode(nullptr)
    , goalThetaNode(nullptr)
    , goalXYZNode(nullptr)
    , travConf(travConf)
    , primitiveConfig(primitiveConfig)
    , mobilityConfig(mobilityConfig)
{
    numAngles = primitiveConfig.numAngles;
    searchGrid.setResolution(Eigen::Vector2d(travConf.gridResolution, travConf.gridResolution));
    searchGrid.extend(travMap->getNumCells());
    robotHalfSize << travConf.robotSizeX / 2, travConf.robotSizeY / 2, travConf.robotHeight/2;
    if(travMap)
    {
        availableMotions.computeMotions(travMap->getResolution().x(), travConf.gridResolution);
    }
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
    searchGrid.clear();

    idToHash.clear();
    travNodeIdToDistance.clear();

    startThetaNode = nullptr;
    startXYZNode = nullptr;

    goalThetaNode = nullptr;
    goalXYZNode = nullptr;

    for(int *p: StateID2IndexMapping)
    {
        delete[] p;
    }
    StateID2IndexMapping.clear();
}



EnvironmentXYZTheta::~EnvironmentXYZTheta()
{
    clear();
}

void EnvironmentXYZTheta::updateMap(shared_ptr<const traversability_generator3d::TravMap3d > travMap)
{
    if(this->travMap && this->travMap->getResolution() != travMap->getResolution()){
        LOG_ERROR_S << "EnvironmentXYZTheta::updateMap : Error got TravMap3d with different resolution";
        throw std::runtime_error("EnvironmentXYZTheta::updateMap : Error got TravMap3d with different resolution");
    }
    if(!this->travMap)
    {
        availableMotions.computeMotions(travMap->getResolution().x(), travConf.gridResolution);
    }
    this->travMap = travMap;

    clear();
}

EnvironmentXYZTheta::XYZNode* EnvironmentXYZTheta::createNewXYZState(traversability_generator3d::TravGenNode* travNode)
{
    XYZNode *xyzNode = new XYZNode(travNode->getHeight(), travNode->getIndex());
    xyzNode->getUserData().travNode = travNode;
    searchGrid.at(travNode->getIndex()).insert(xyzNode);

    return xyzNode;
}

EnvironmentXYZTheta::ThetaNode* EnvironmentXYZTheta::createNewStateFromPose(const std::string &name, const Eigen::Vector3d& pos, double theta, XYZNode **xyzBackNode)
{
    traversability_generator3d::TravGenNode* travNode = findMatchingTraversabilityPatchAt(pos);

    if(!travNode)
    {
        LOG_ERROR_S << "Error, could not find matching trav node for " << name;
        return nullptr;
    }

    if(!travNode->isExpanded())
    {
        LOG_ERROR_S << "createNewStateFromPose: Error: " << name << " Pose " << pos.transpose() << " is not traversable";
        return nullptr;
    }

    XYZNode *xyzNode = createNewXYZState(travNode);

    DiscreteTheta thetaD(theta, numAngles);

    if(xyzBackNode)
        *xyzBackNode = xyzNode;

    return createNewState(thetaD, xyzNode);
}

traversability_generator3d::TravGenNode* EnvironmentXYZTheta::findMatchingTraversabilityPatchAt(const Eigen::Vector3d& pos){
    maps::grid::Index idxTravNode;
    if(!travMap->toGrid(pos, idxTravNode))
    {
        LOG_ERROR_S << "EnvironmentXYZTheta::findMatchingTraversabilityPatchAt: Position outside of map !";
        return nullptr;
    }

    auto &trList(travMap->at(idxTravNode));

    //check if we got an existing node
    for(traversability_generator3d::TravGenNode *snode : trList)
    {
        const double searchHeight = snode->getHeight();

        if((searchHeight - travConf.maxStepHeight) <= pos.z() && (searchHeight + travConf.maxStepHeight) >= pos.z())
        {
            //found a connectable node
            return snode;
        }

        if(searchHeight > pos.z())
        {
            return nullptr;
        }
    }
    return nullptr;        
}

bool EnvironmentXYZTheta::obstacleCheck(const maps::grid::Vector3d& pos, double theta,
                                        const traversability_generator3d::TraversabilityConfig& travConf,
                                        const SplinePrimitivesConfig& splineConf,
                                        const std::string& nodeName)
{
    traversability_generator3d::TravGenNode* travNode = findMatchingTraversabilityPatchAt(pos);
    if(!travNode)
    {
        LOG_ERROR_S << "Error, could not find matching trav node for " << nodeName;
        return false;
    }

    LOG_ERROR_S << "NodeType: " << travNode->getUserData().nodeType;

    if (travNode->getUserData().nodeType != ::traversability_generator3d::NodeType::TRAVERSABLE)
    {
        return false;
    }

    if (usePathStatistics){
        PathStatistic stats(travConf);
        std::vector<base::Pose2D> poses;
        std::vector<const traversability_generator3d::TravGenNode*> path;
        path.push_back(travNode);

        const Eigen::Vector3d centeredPos = travNode->getPosition(*travMap);


        //NOTE theta needs to be discretized because the planner uses discrete theta internally everywhere.
        //     If we do not discretize here, external calls and internal calls will have different results for the same pose input

        DiscreteTheta discTheta(theta, splineConf.numAngles);

        poses.push_back(base::Pose2D(centeredPos.topRows(2), discTheta.getRadian()));

        stats.calculateStatistics(path, poses, *travMap, "ugv_nav4d_" + nodeName + "Box");

        if(stats.getRobotStats().getNumObstacles() || stats.getRobotStats().getNumFrontiers()) 
        {
#ifdef ENABLE_DEBUG_DRAWINGS
            V3DD::COMPLEX_DRAWING([&]()
            {
                const std::string drawName("ugv_nav4d_obs_check_fail_" + nodeName);
                V3DD::CLEAR_DRAWING(drawName);
                V3DD::DRAW_WIREFRAME_BOX(drawName, pos, Eigen::Quaterniond(Eigen::AngleAxisd(discTheta.getRadian(), Eigen::Vector3d::UnitZ())), Eigen::Vector3d(travConf.robotSizeX, travConf.robotSizeY, travConf.robotHeight), V3DD::Color::red);
            });
#endif

            LOG_DEBUG_S << "Num obstacles: " << stats.getRobotStats().getNumObstacles();
            LOG_DEBUG_S << "Error: " << nodeName << " inside obstacle";
            return false;
        }
    }
    return true;
}

bool EnvironmentXYZTheta::checkStartGoalNode(const string& name, traversability_generator3d::TravGenNode *node, double theta)
{
    //check for collisions NOTE has to be done after expansion

    maps::grid::Vector3d nodePos;
    travMap->fromGrid(node->getIndex(), nodePos, node->getHeight(), false);
#ifdef ENABLE_DEBUG_DRAWINGS
        V3DD::COMPLEX_DRAWING([&]()
        {
            const std::string drawName("ugv_nav4d_check_start_goal_" + name);
            V3DD::CLEAR_DRAWING(drawName);
            V3DD::DRAW_WIREFRAME_BOX(drawName, nodePos, Eigen::Quaterniond(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ())), Eigen::Vector3d(travConf.robotSizeX, travConf.robotSizeY, travConf.robotHeight), V3DD::Color::red);
        });
#endif


    return obstacleCheck(nodePos, theta, travConf, primitiveConfig, name);
}

void EnvironmentXYZTheta::setGoal(const Eigen::Vector3d& goalPos, double theta)
{

#ifdef ENABLE_DEBUG_DRAWINGS
    V3DD::CLEAR_DRAWING("ugv_nav4d_env_goalPos");
    V3DD::DRAW_ARROW("ugv_nav4d_env_goalPos", goalPos, base::Quaterniond(Eigen::AngleAxisd(M_PI, base::Vector3d::UnitX())),
            base::Vector3d(1,1,1), V3DD::Color::red);
#endif

    LOG_DEBUG_S << "GOAL IS: " << goalPos.transpose();

    if(!startXYZNode)
        throw std::runtime_error("Error, start needs to be set before goal");

    goalThetaNode = createNewStateFromPose("goal", goalPos, theta, &goalXYZNode);
    if(!goalThetaNode)
    {
        throw StateCreationFailed("Failed to create goal state");
    }

    const auto nodeType = goalXYZNode->getUserData().travNode->getUserData().nodeType;
    if(nodeType != maps::grid::TraversabilityNodeBase::TRAVERSABLE) {
        throw std::runtime_error("Error, goal has to be a traversable patch");
    }


    if(travConf.enableInclineLimitting)
    {
        if(!checkOrientationAllowed(goalXYZNode->getUserData().travNode, theta))
        {
            throw OrientationNotAllowed("Goal orientation not allowed due to slope");
        }
    }


    //NOTE If we want to precompute the heuristic (precomputeCost()) we need to expand
    //     the whole travmap beforehand.

    //check goal position
    if(!checkStartGoalNode("goal", goalXYZNode->getUserData().travNode, goalThetaNode->theta.getRadian()))
    {
        throw ObstacleCheckFailed("goal position is invalid");
    }

    precomputeCost();

    //draw greedy path
#ifdef ENABLE_DEBUG_DRAWINGS
    V3DD::COMPLEX_DRAWING([&]()
    {
        V3DD::CLEAR_DRAWING("ugv_nav4d_greedyPath");
        traversability_generator3d::TravGenNode* nextNode = startXYZNode->getUserData().travNode;
        traversability_generator3d::TravGenNode* goal = goalXYZNode->getUserData().travNode;
        while(nextNode != goal)
        {
            maps::grid::Vector3d pos;
            travMap->fromGrid(nextNode->getIndex(), pos, nextNode->getHeight(), false);

            V3DD::DRAW_CYLINDER("ugv_nav4d_greedyPath", pos, base::Vector3d(0.03, 0.03, 0.3), V3DD::Color::yellow);
            double minCost = std::numeric_limits< double >::max();
            bool foundNextNode = false;
            for(maps::grid::TraversabilityNodeBase* node : nextNode->getConnections())
            {
                traversability_generator3d::TravGenNode* travNode = static_cast<traversability_generator3d::TravGenNode*>(node);
                const double cost = travNodeIdToDistance[travNode->getUserData().id].distToGoal;
                if(cost < minCost)
                {
                    minCost = cost;
                    nextNode = travNode;
                    foundNextNode = true;
                }
            }
            if (!foundNextNode) {
                LOG_DEBUG_S << "nextNode has no connection";
                break;
            }
        }
    });
#endif
}

void EnvironmentXYZTheta::setStart(const Eigen::Vector3d& startPos, double theta)
{
#ifdef ENABLE_DEBUG_DRAWINGS
        V3DD::CLEAR_DRAWING("ugv_nav4d_env_startPos");
        V3DD::DRAW_ARROW("ugv_nav4d_env_startPos", startPos, base::Quaterniond(Eigen::AngleAxisd(M_PI, base::Vector3d::UnitX())),
                     base::Vector3d(1,1,1), V3DD::Color::blue);
#endif

    LOG_DEBUG_S << "START IS: " << startPos.transpose();

    startThetaNode = createNewStateFromPose("start", startPos, theta, &startXYZNode);
    if(!startThetaNode){
        LOG_ERROR_S << "Failed to create start state";
        throw StateCreationFailed("Failed to create start state");
    }

    //check start position
    if(!checkStartGoalNode("start", startXYZNode->getUserData().travNode, startThetaNode->theta.getRadian()))
    {
        LOG_ERROR_S<< "Start position is invalid";
        throw ObstacleCheckFailed("Start position inside obstacle");
    }
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
    //sbpl never calls this
    throw std::runtime_error("GetFromToHeuristic not implemented");
}

maps::grid::Vector3d EnvironmentXYZTheta::getStatePosition(const int stateID) const
{
    const Hash &sourceHash(idToHash[stateID]);
    const XYZNode *node = sourceHash.node;
    maps::grid::Vector3d ret;
    travMap->fromGrid(node->getIndex(), ret, node->getHeight());
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

    if(cost == -1){
        LOG_ERROR_S << "Internal Error: No matching motion for output path found";
        throw std::runtime_error("Internal Error: No matching motion for output path found");
    }
    return availableMotions.getMotion(motionId);
}


int EnvironmentXYZTheta::GetGoalHeuristic(int stateID)
{

    // the heuristic distance has been calculated beforehand. Here it is just converted to
    // travel time.

    const Hash &sourceHash(idToHash[stateID]);
    const XYZNode *sourceNode = sourceHash.node;
    const traversability_generator3d::TravGenNode* travNode = sourceNode->getUserData().travNode;
    const ThetaNode *sourceThetaNode = sourceHash.thetaNode;

    if(travNode->getUserData().nodeType != ::traversability_generator3d::NodeType::TRAVERSABLE)
    {
        return std::numeric_limits<int>::max();
    }

    const double sourceToGoalDist = travNodeIdToDistance[travNode->getUserData().id].distToGoal;
    const double timeTranslation = sourceToGoalDist / mobilityConfig.translationSpeed;

    //for point turns the translational time is zero, however turning still takes time
    const double timeRotation = sourceThetaNode->theta.shortestDist(goalThetaNode->theta).getRadian() / mobilityConfig.rotationSpeed;

    //scale by costScaleFactor to avoid loss of precision before converting to int
    const double maxTime = std::max(timeTranslation, timeRotation);

    // try to avoid overflow by skipping scaling for already large values (scaling is only useful for small values)
    int result = maxTime >= 10000000 ? maxTime : maxTime * Motion::costScaleFactor;
    if(result < 0)
    {
        LOG_ERROR_S << sourceToGoalDist;
        LOG_ERROR_S << stateID;
        LOG_ERROR_S << mobilityConfig.translationSpeed;
        LOG_ERROR_S << timeTranslation;
        LOG_ERROR_S << sourceThetaNode->theta.shortestDist(goalThetaNode->theta).getRadian();
        LOG_ERROR_S << mobilityConfig.rotationSpeed;
        LOG_ERROR_S << timeRotation;
        LOG_ERROR_S << result;
        LOG_ERROR_S << travNode->getUserData().id;
        LOG_ERROR_S << travNode->getUserData().nodeType;
        //throw std::runtime_error("Goal heuristic < 0");
        LOG_ERROR_S<< "Overflow while computing goal heuristic!";
        result = std::numeric_limits<int>::max();
    }
    oassert(result >= 0);
    return result;
}

void EnvironmentXYZTheta::enablePathStatistics(bool enable){
    usePathStatistics = enable;
}

int EnvironmentXYZTheta::GetStartHeuristic(int stateID)
{
    const Hash &targetHash(idToHash[stateID]);
    const XYZNode *targetNode = targetHash.node;
    const traversability_generator3d::TravGenNode* travNode = targetNode->getUserData().travNode;
    const ThetaNode *targetThetaNode = targetHash.thetaNode;

    const double startToTargetDist = travNodeIdToDistance[travNode->getUserData().id].distToStart;
    const double timeTranslation = startToTargetDist / mobilityConfig.translationSpeed;
    double timeRotation = startThetaNode->theta.shortestDist(targetThetaNode->theta).getRadian() / mobilityConfig.rotationSpeed;

    const int result = floor(std::max(timeTranslation, timeRotation) * Motion::costScaleFactor);
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

traversability_generator3d::TravGenNode *EnvironmentXYZTheta::movementPossible(traversability_generator3d::TravGenNode *fromTravNode, const maps::grid::Index &fromIdx, const maps::grid::Index &toIdx)
{
    if(toIdx == fromIdx)
        return fromTravNode;

    //get trav node associated with the next index
    traversability_generator3d::TravGenNode *targetNode = fromTravNode->getConnectedNode(toIdx);
    if(!targetNode)
    {
        //FIXME this should never happen but it did happen in the past and I have no idea why
        //      needs investigation!
        LOG_DEBUG_S<< "Movement not possible. Nodes are not connected";
        return nullptr;
    }

    if(!checkExpandTreadSafe(targetNode))
    {
        return nullptr;
    }

    //NOTE this check cannot be done before checkExpandTreadSafe because the type will be determined
    //     during the expansion. Beforehand the type is undefined
    if(targetNode->getUserData().nodeType != ::traversability_generator3d::NodeType::TRAVERSABLE)
    {
        LOG_DEBUG_S<< "movement not possible. targetnode not traversable";
        return nullptr;
    }
    return targetNode;
}

bool EnvironmentXYZTheta::checkExpandTreadSafe(traversability_generator3d::TravGenNode * node)
{
    if(node->isExpanded())
    {
        return true;
    }
    return false;
}


void EnvironmentXYZTheta::GetSuccs(int SourceStateID, vector< int >* SuccIDV, vector< int >* CostV)
{
    std::vector<size_t> motionId;
    GetSuccs(SourceStateID, SuccIDV, CostV, motionId);
}


traversability_generator3d::TravGenNode * EnvironmentXYZTheta::checkTraversableHeuristic(const maps::grid::Index sourceIndex, traversability_generator3d::TravGenNode *sourceNode,
                                                             const Motion &motion, const maps::grid::TraversabilityMap3d<traversability_generator3d::TravGenNode *> &trMap)
{
    traversability_generator3d::TravGenNode *travNode = sourceNode;

    maps::grid::Index curIndex = sourceIndex;
    for(const PoseWithCell &diff : motion.intermediateStepsTravMap)
    {
        //diff is always a full offset to the start position
        const maps::grid::Index newIndex =  sourceIndex + diff.cell;
        travNode = movementPossible(travNode, curIndex, newIndex);
        if(!travNode)
        {
            return nullptr;
        }

        curIndex = newIndex;
    }

    return travNode;
}

void EnvironmentXYZTheta::GetSuccs(int SourceStateID, vector< int >* SuccIDV, vector< int >* CostV, vector< size_t >& motionIdV)
{
    SuccIDV->clear();
    CostV->clear();
    motionIdV.clear();
    const Hash &sourceHash(idToHash[SourceStateID]);
    const XYZNode *const sourceNode = sourceHash.node;
    const ThetaNode *const sourceThetaNode = sourceHash.thetaNode;
    traversability_generator3d::TravGenNode *sourceTravNode = sourceNode->getUserData().travNode;

#ifdef ENABLE_DEBUG_DRAWINGS
        V3DD::COMPLEX_DRAWING([&]()
        {

            const traversability_generator3d::TravGenNode* node = sourceNode->getUserData().travNode;
            Eigen::Vector3d pos((node->getIndex().x() + 0.5) * travConf.gridResolution,
                                (node->getIndex().y() + 0.5) * travConf.gridResolution,
                                node->getHeight());
            pos = travMap->getLocalFrame().inverse(Eigen::Isometry) * pos;
            V3DD::DRAW_WIREFRAME_BOX("ugv_nav4d_successors", pos, base::Vector3d(travMap->getResolution().x() / 2.0, travMap->getResolution().y() / 2.0,
                            0.05), V3DD::Color::blue);
        });
#endif

    if(!sourceTravNode->isExpanded())
    {
        //expansion failed, current node is not driveable -> there are not successors to this state
        LOG_DEBUG_S<< "GetSuccs: current node not expanded and not expandable";
        return;
    }

    Eigen::Vector3d sourcePosWorld;
    travMap->fromGrid(sourceNode->getIndex(), sourcePosWorld, sourceTravNode->getHeight(), false);

    const auto& motions = availableMotions.getMotionForStartTheta(sourceThetaNode->theta);

    //dynamic scheduling is choosen because the iterations have vastly different runtime
    //due to the different sanity checks
    //the chunk size (5) was chosen to reduce dynamic scheduling overhead.
    //**No** tests have been done to verify whether 5 is a good value or not!
    #pragma omp parallel for schedule(dynamic, 5)
    for(size_t i = 0; i < motions.size(); ++i)
    {
        //check that the motion is traversable (without collision checks) and find the goal node of the motion
        const ugv_nav4d::Motion &motion(motions[i]);
        traversability_generator3d::TravGenNode *goalTravNode = checkTraversableHeuristic(sourceNode->getIndex(), sourceNode->getUserData().travNode, motions[i], *travMap);
        if(!goalTravNode)
        {
            //at least one node on the path is not traversable
            continue;
        }

        std::vector<const traversability_generator3d::TravGenNode*> nodesOnTravPath;
        std::vector<base::Pose2D> posesOnPath;
        maps::grid::Index curIdx = sourceTravNode->getIndex();
        traversability_generator3d::TravGenNode *travNode = sourceTravNode;
        bool intermediateStepsOk = true;
        for(const PoseWithCell &diff : motion.intermediateStepsObstMap)
        {
            //diff is always a full offset to the start position
            const maps::grid::Index newIndex =  sourceTravNode->getIndex() + diff.cell;
            travNode = movementPossible(travNode, curIdx, newIndex);
            nodesOnTravPath.push_back(travNode);
            base::Pose2D curPose = diff.pose;
            curPose.position += sourcePosWorld.head<2>();
            posesOnPath.push_back(curPose);
            if(!travNode)
            {
                intermediateStepsOk = false;
                break;
            }

            if(travConf.enableInclineLimitting)
            {
                if(!checkOrientationAllowed(travNode, diff.pose.orientation))
                {
                    intermediateStepsOk = false;
                    break;
                }
            }
            curIdx = newIndex;
        }

        //no way from start to end on trav map
        if(!intermediateStepsOk)
            continue;

        if (usePathStatistics){
            PathStatistic statistic(travConf);

            if(!statistic.isPathFeasible(nodesOnTravPath, posesOnPath, *getTraversabilityMap()))
            {
                continue;
            }
        }

        //goal from source to the end of the motion was valid
        XYZNode *successXYNode = nullptr;
        ThetaNode *successthetaNode = nullptr;

        //WARNING This becomes a critical section if several motion primitives
        //        share the same finalPos.
        //        As long as this is not the case this section should be save.
        const maps::grid::Index finalPos(sourceNode->getIndex() + maps::grid::Index(motion.xDiff,motion.yDiff));

        #pragma omp critical(searchGridAccess)
        {
            const auto &candidateMap = searchGrid.at(finalPos);

            if(goalTravNode->getIndex() != finalPos){
                LOG_ERROR_S << "Internal error, indexes of goalTravNode and finalPos do not match";
                throw std::runtime_error("Internal error, indexes of goalTravNode and finalPos do not match");
            }
            XYZNode searchTmp(goalTravNode->getHeight(), goalTravNode->getIndex());

            //this works, as the equals check is on the height, not the node itself
            auto it = candidateMap.find(&searchTmp);

            if(it != candidateMap.end())
            {
                //found a node with a matching height
                successXYNode = *it;
            }
            else
            {
                successXYNode = createNewXYZState(goalTravNode); //modifies searchGrid at travNode->getIndex()
            }
        }

        #pragma omp critical(thetaToNodesAccess)
        {
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
        }

        double cost = 0;
        switch(travConf.slopeMetric)
        {
            case traversability_generator3d::SlopeMetric::AVG_SLOPE:
            {
                double avgSlope = 0;
                if(nodesOnTravPath.size() > 0)
                {
                    avgSlope = getAvgSlope(nodesOnTravPath);
                }
                else
                {
                    //This happens on point turns as they have no intermediate steps
                    avgSlope = sourceTravNode->getUserData().slope;
                }
                const double slopeFactor = avgSlope * travConf.slopeMetricScale;
                cost = motion.baseCost + motion.baseCost * slopeFactor;
                break;
            }
            case traversability_generator3d::SlopeMetric::MAX_SLOPE:
            {
                double maxSlope = 0;
                if(nodesOnTravPath.size() > 0)
                {
                    maxSlope = getMaxSlope(nodesOnTravPath);
                }
                else
                {
                    //This happens on point turns as they have no intermediate steps
                    maxSlope = sourceTravNode->getUserData().slope;
                }
                const double slopeFactor = maxSlope * travConf.slopeMetricScale;
                cost = motion.baseCost + motion.baseCost * slopeFactor;
                break;
            }
            case traversability_generator3d::SlopeMetric::TRIANGLE_SLOPE:
            {
                //assume that the motion is a straight line, extrapolate into third dimension
                //by projecting onto a plane that connects start and end cell.
                const double heightDiff = std::abs(sourceNode->getHeight() - successXYNode->getHeight());
                //not perfect but probably more exact than the slope factors above
                const double approxMotionLen3D = std::sqrt(std::pow(motion.translationlDist, 2) + std::pow(heightDiff, 2));
                assert(approxMotionLen3D >= motion.translationlDist);//due to triangle inequality
                const double translationalVelocity = mobilityConfig.translationSpeed;
                cost = Motion::calculateCost(approxMotionLen3D, motion.angularDist, translationalVelocity,
                                             mobilityConfig.rotationSpeed, motion.costMultiplier);
                break;
            }
            case traversability_generator3d::SlopeMetric::NONE:
                cost = motion.baseCost;
                break;
            default:
                LOG_ERROR_S << "Unknown slope metric selected";
                throw std::runtime_error("Unknown slope metric selected");
        }

        if (usePathStatistics){
            PathStatistic statistic(travConf);
            if(statistic.getBoundaryStats().getNumObstacles())
            {
                const double outer_radius = travConf.costFunctionDist;
                double minDistToRobot = statistic.getBoundaryStats().getMinDistToObstacles();
                minDistToRobot = std::min(outer_radius, minDistToRobot);
                double impactFactor = (outer_radius - minDistToRobot) / outer_radius;
                oassert(impactFactor < 1.001 && impactFactor >= 0);

                cost += cost * impactFactor;
            }

            if(statistic.getBoundaryStats().getNumFrontiers())
            {
                const double outer_radius = travConf.costFunctionDist;
                double minDistToRobot = statistic.getBoundaryStats().getMinDistToFrontiers();
                minDistToRobot = std::min(outer_radius, minDistToRobot);
                double impactFactor = (outer_radius - minDistToRobot) / outer_radius;
                oassert(impactFactor < 1.001 && impactFactor >= 0);

                cost += cost * impactFactor;
            }
        }

        oassert(cost <= std::numeric_limits<int>::max() && cost >= std::numeric_limits< int >::min());
        oassert(int(cost) >= motion.baseCost);
        oassert(motion.baseCost > 0);

        const int iCost = (int)cost;
        #pragma omp critical(updateData)
        {
            SuccIDV->push_back(successthetaNode->id);
            CostV->push_back(iCost);
            motionIdV.push_back(motion.id);

            //####BEGIN DEBUG BLOCK!
            {
                const Hash &sourceHashh(idToHash[successthetaNode->id]);
                const XYZNode *sourceNodeh = sourceHashh.node;
                const traversability_generator3d::TravGenNode* travNodeh = sourceNodeh->getUserData().travNode;

                if(travNodeh->getType() != maps::grid::TraversabilityNodeBase::TRAVERSABLE)
                {
                    LOG_ERROR_S << "In GetSuccs() returned id for non-traversable patch";
                    throw std::runtime_error("In GetSuccs() returned id for non-traversable patch");
                }
            }
            //####END DEBUG BLOCK!!!
        }
    }
}

bool EnvironmentXYZTheta::checkOrientationAllowed(const traversability_generator3d::TravGenNode* node,
                                const base::Orientation2D& orientationRad) const
{
    //otherwise something went wrong when generating the map
    assert(node->getUserData().allowedOrientations.size() > 0);

    const base::Angle orientation = base::Angle::fromRad(orientationRad);
    bool isInside = false;
    for(const base::AngleSegment& segment : node->getUserData().allowedOrientations)
    {
        if(segment.isInside(orientation))
        {
            isInside = true;
            break;
        }
    }
    return isInside;
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
        LOG_INFO_S<<  buffer.str();

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

void EnvironmentXYZTheta::getTrajectory(const vector<int>& stateIDPath,
                                        vector<SubTrajectory>& result,
                                        bool setZToZero, const Eigen::Vector3d &startPos,
                                        const Eigen::Vector3d &goalPos, const double& goalHeading, const Eigen::Affine3d &plan2Body)
{
    if(stateIDPath.size() < 2)
        return;

    result.clear();
    base::Trajectory curPart;

#ifdef ENABLE_DEBUG_DRAWINGS
        V3DD::CLEAR_DRAWING("ugv_nav4d_trajectory");
#endif

    size_t indexOfMotionToUpdate{stateIDPath.size()-2};
    const Motion& finalMotion = getMotion(stateIDPath[stateIDPath.size()-2], stateIDPath[stateIDPath.size()-1]);
    if (finalMotion.type == Motion::Type::MOV_POINTTURN && stateIDPath.size() > 2){ //assuming that there are no consecutive point turns motion at the end of a planned trajectory
        indexOfMotionToUpdate = stateIDPath.size()-3;
    }

    bool updateGoalPose = false;
    Eigen::Hyperplane<double, 3> travNodePlane;
    
    Eigen::Vector3d start = startPos;
    for(size_t i = 0; i < stateIDPath.size() - 1; ++i)
    {
        const Motion& curMotion = getMotion(stateIDPath[i], stateIDPath[i+1]);
        const Hash &startHash(idToHash[stateIDPath[i]]);
        const maps::grid::Index startIndex(startHash.node->getIndex());
        maps::grid::Index lastIndex = startIndex;
        traversability_generator3d::TravGenNode *curNode = startHash.node->getUserData().travNode;
        std::vector<base::Vector3d> positions;

        for(const CellWithPoses &cwp : curMotion.fullSplineSamples)
        {
            maps::grid::Index curIndex = startIndex + cwp.cell;
            if(curIndex != lastIndex)
            {
                traversability_generator3d::TravGenNode *nextNode = curNode->getConnectedNode(curIndex);
                if(!nextNode)
                {
                    LOG_ERROR_S << "Internal error, trajectory is not continuous on traversability grid";
                    throw std::runtime_error("Internal error, trajectory is not continuous on traversability grid");
                }
                curNode = nextNode;
                lastIndex = curIndex;
            }

            Eigen::Vector3d posWorld;
            travMap->fromGrid(curNode->getIndex(), posWorld, curNode->getHeight(), false);

            // Set up the plane at the 3D world position
            travNodePlane.normal() = curNode->getUserData().plane.normal();
            travNodePlane.offset() = -travNodePlane.normal().dot(posWorld); // Align the plane offset to posWorld

            for (const base::Pose2D &p : cwp.poses)
            {
                Eigen::Vector3d point{p.position.x(), p.position.y(), 0};
                Eigen::Vector3d globalPoint = point + start;
                Eigen::ParametrizedLine<double, 3> line = Eigen::ParametrizedLine<double, 3>::Through(globalPoint, globalPoint + Eigen::Vector3d::UnitZ());
                Eigen::Vector3d pointOnTravPlane = line.intersectionPoint(travNodePlane); 
#ifdef ENABLE_DEBUG_DRAWINGS
                V3DD::DRAW_SPHERE("ugv_nav4d_trajectory_poses", pointOnTravPlane, 0.01, V3DD::Color::red);
#endif
                //TODO: Only left here until software which still uses trajectory2D is updated to use trajectory3D
                if (setZToZero){
                    pointOnTravPlane.z() = 0;
                }

                Eigen::Vector3d pointOnBody = plan2Body.inverse(Eigen::Isometry) * pointOnTravPlane;
                if (positions.empty() || !(positions.back().isApprox(pointOnBody)))
                {
                    positions.emplace_back(pointOnBody);
                }
            }
        }
        if (mobilityConfig.remove_goal_offset == true &&
            i == indexOfMotionToUpdate)
        {
            double goal_offset_x = (goalPos.x() - positions[positions.size()-1].x()) / (positions.size()-1);
            double goal_offset_y = (goalPos.y() - positions[positions.size()-1].y()) / (positions.size()-1);

            for (std::size_t j{0}; j < positions.size(); j++){
                positions[j].x() += j*goal_offset_x;
                positions[j].y() += j*goal_offset_y;
            }
            updateGoalPose = true;
        }

        curPart.spline.interpolate(positions);

#ifdef ENABLE_DEBUG_DRAWINGS
            V3DD::COMPLEX_DRAWING([&]()
            {
                Eigen::Vector4d color = V3DD::Color::cyan;
                Eigen::Vector3d size(0.01, 0.01, 0.2);
                switch(curMotion.type)
                {
                    case Motion::MOV_BACKWARD:
                        color = V3DD::Color::magenta;
                        break;
                    case Motion::MOV_FORWARD:
                        color = V3DD::Color::cyan;
                        break;
                    case Motion::MOV_POINTTURN:
                        color = V3DD::Color::red;
                        size.z() = 1;
                        V3DD::DRAW_CYLINDER("ugv_nav4d_trajectory", getStatePosition(stateIDPath[i]),  size, color);
                        break;
                    case Motion::MOV_LATERAL:
                        color = V3DD::Color::green;
                        break;

                    default:
                        color =  V3DD::Color::red;
                }
                for(base::Vector3d pos : positions)
                {
    //                 pos = travMap->getLocalFrame().inverse(Eigen::Isometry) * pos;
                    V3DD::DRAW_CYLINDER("ugv_nav4d_trajectory", pos,  size, color);
                }
            });
#endif

        if (curMotion.type == Motion::Type::MOV_POINTTURN)
        {
            SubTrajectory subtraj;
            subtraj.driveMode = DriveMode::ModeTurnOnTheSpot;

            std::vector<base::Angle> angles;
            angles.emplace_back(base::Angle::fromRad(curMotion.startTheta.getRadian()));
            angles.emplace_back(base::Angle::fromRad(curMotion.endTheta.getRadian()));

            base::Pose2D startPose;
            startPose.position.x() = start.x();
            startPose.position.y() = start.y();
            startPose.orientation  = curMotion.startTheta.getRadian();

            base::Pose2D goalPose;
            goalPose.position.x() = start.x();
            goalPose.position.y() = start.y();
            goalPose.orientation  = curMotion.endTheta.getRadian();

            subtraj.interpolate(startPose,angles);
            subtraj.startPose     = startPose;
            subtraj.goalPose      = goalPose;
            result.push_back(subtraj);
        }
        else
        {
            SubTrajectory curPartSub(curPart);
            curPartSub.speed = (curMotion.type == Motion::Type::MOV_BACKWARD) ? -mobilityConfig.translationSpeed : mobilityConfig.translationSpeed;
            curPartSub.driveMode = (curMotion.type == Motion::Type::MOV_LATERAL) ? DriveMode::ModeSideways : DriveMode::ModeAckermann;

            result.push_back(curPartSub);

            if (updateGoalPose){
                SubTrajectory subtraj;
                subtraj.driveMode = DriveMode::ModeTurnOnTheSpot;

                base::Pose2D startPose;
                startPose.position.x() = curPart.spline.getEndPoint().x();
                startPose.position.y() = curPart.spline.getEndPoint().y();
                startPose.orientation  = curPart.spline.getHeading(curPart.spline.getEndParam());
                if (startPose.orientation < 0){
                    startPose.orientation += 2*M_PI;
                }

                base::Pose2D goalPose;
                goalPose.position.x() = curPart.spline.getEndPoint().x();
                goalPose.position.y() = curPart.spline.getEndPoint().y();
                goalPose.orientation  = goalHeading;
                if (goalPose.orientation < 0){
                    goalPose.orientation += 2*M_PI;
                }
                
                if (std::abs(goalPose.orientation - startPose.orientation) > 0.01){ //needed otherwise spline interpolation has an exception
                    std::vector<base::Angle> angles;
                    angles.emplace_back(base::Angle::fromRad(startPose.orientation));
                    angles.emplace_back(base::Angle::fromRad(goalPose.orientation));

                    subtraj.interpolate(goalPose,angles);
                    subtraj.startPose     = startPose;
                    subtraj.goalPose      = goalPose;
                    result.push_back(subtraj);
                }
                updateGoalPose = false;
            }
            start = curPart.spline.getEndPoint();
        }
    }
}

const std::shared_ptr<const traversability_generator3d::TravMap3d > EnvironmentXYZTheta::getTraversabilityMap() const
{
    return travMap;
}

const PreComputedMotions& EnvironmentXYZTheta::getAvailableMotions() const
{
    return availableMotions;
}

double EnvironmentXYZTheta::getAvgSlope(std::vector<const traversability_generator3d::TravGenNode*> path) const
{
    if(path.size() <= 0)
    {
        LOG_ERROR_S << "Requested slope of path with length zero.";
        throw std::runtime_error("Requested slope of path with length zero.");
    }
    double slopeSum = 0;
    for(const traversability_generator3d::TravGenNode* node : path)
    {
        slopeSum += node->getUserData().slope;
    }
    const double avgSlope = slopeSum / path.size();
    return avgSlope;
}

double EnvironmentXYZTheta::getMaxSlope(std::vector<const traversability_generator3d::TravGenNode*> path) const
{
    const traversability_generator3d::TravGenNode* maxElem =  *std::max_element(path.begin(), path.end(),
                                  [] (const traversability_generator3d::TravGenNode* lhs, const traversability_generator3d::TravGenNode* rhs)
                                  {
                                    return lhs->getUserData().slope < rhs->getUserData().slope;
                                  });
    return maxElem->getUserData().slope;
}

void EnvironmentXYZTheta::precomputeCost()
{
    std::unordered_map<const maps::grid::TraversabilityNodeBase*, double> costToStart;
    std::unordered_map<const maps::grid::TraversabilityNodeBase*, double> costToEnd;

    // Compute costs
    Dijkstra::computeCost(startXYZNode->getUserData().travNode, costToStart, travConf);
    Dijkstra::computeCost(goalXYZNode->getUserData().travNode, costToEnd, travConf);

    // Validate keys in both maps
    if (costToStart.size() != costToEnd.size()) {
        throw std::runtime_error("Mismatch: costToStart size(" + std::to_string(costToStart.size()) + ")" 
                                + " and costToEnd size(" + std::to_string(costToEnd.size()) + ")" 
                                + " have different sizes.");
    }

    size_t largestId = 0; // Assuming IDs are non-negative, or use an appropriate minimum value
    for (const maps::grid::LevelList<traversability_generator3d::TravGenNode *> &l : *travMap) {
        for (traversability_generator3d::TravGenNode *n : l) {
            if (n != nullptr) { // Safety check
                largestId = std::max(largestId, n->getUserData().id);
            }
        }
    }

    // Initialize distances
    const double maxDist = std::numeric_limits<double>::max(); // Use a meaningful constant
    travNodeIdToDistance.clear();
    travNodeIdToDistance.resize(largestId, Distance(maxDist, maxDist));

    // Process costToStart
    for (const auto& pair : costToStart) {
        const auto* node = static_cast<const traversability_generator3d::TravGenNode*>(pair.first);
        if (!node) {
            throw std::runtime_error("Invalid node encountered in costToStart.");
        }
        const size_t nodeId = node->getUserData().id;
        travNodeIdToDistance[nodeId].distToStart = pair.second;
    }

    // Process costToEnd
    for (const auto& pair : costToEnd) {
        const auto* node = static_cast<const traversability_generator3d::TravGenNode*>(pair.first);
        if (!node) {
            throw std::runtime_error("Invalid node encountered in costToEnd.");
        }
        const size_t nodeId = node->getUserData().id;
        travNodeIdToDistance[nodeId].distToGoal = pair.second;
    }
}

void EnvironmentXYZTheta::setTravConfig(const traversability_generator3d::TraversabilityConfig& cfg)
{
    travConf = cfg;
}

std::shared_ptr<SubTrajectory> EnvironmentXYZTheta::findTrajectoryOutOfObstacle(const Eigen::Vector3d& start,
                                                                                double theta,
                                                                                const Eigen::Affine3d& ground2Body,
                                                                                bool setZToZero)
{
    traversability_generator3d::TravGenNode* startTravNode = findMatchingTraversabilityPatchAt(start);
    if(!startTravNode)
    {
        LOG_ERROR_S<< "EnvironmentXYZTheta::findTrajectoryOutOfObstacle(): Unable to generate trav node corresponding to start position";
        throw std::runtime_error("EnvironmentXYZTheta::findTrajectoryOutOfObstacle(): Unable to generate trav node corresponding to start position");
    }

    if(!startTravNode->isExpanded())
    {
        //this node should be expanded
        LOG_ERROR_S<< "EnvironmentXYZTheta::findTrajectoryOutOfObstacle(): Start position is not expanded!";
        throw std::runtime_error("EnvironmentXYZTheta::findTrajectoryOutOfObstacle(): Start position is not expanded!");        
    }

    Eigen::Vector3d startPosWorld;
    travMap->fromGrid(startTravNode->getIndex(), startPosWorld, startTravNode->getHeight(), false);

    DiscreteTheta thetaD(theta, numAngles);
    const maps::grid::Index startIdxTravMap =  startTravNode->getIndex();

    int bestMotionIndex = -1;
    std::vector<const traversability_generator3d::TravGenNode*> bestNodesOnPath;
    std::vector<base::Pose2D> bestPosesOnObstPath;
    int bestMotionObstacleCount = std::numeric_limits<int>::max();

    bool intermediateStepsOk = true;
    const auto& motions = availableMotions.getMotionForStartTheta(thetaD);
    for(size_t i = 0; i < motions.size(); ++i)
    {
        const ugv_nav4d::Motion &motion(motions[i]);
        const traversability_generator3d::TravGenNode* currentNode = startTravNode;
        std::vector<const traversability_generator3d::TravGenNode*> nodesOnPath;
        std::vector<base::Pose2D> posesOnPath;

        nodesOnPath.push_back(currentNode);

        base::Pose2D firstPose;

        //Currently the function only selects a single motion so the pointturn will not help us.
        //TODO: If multiple motions are agreegated to get the final recovery trajectory then pointturns can be used.
        //NOTE: Pointturns have no intermediateStepsObstMap, so are skipped at the moment.
        if (motion.type == ugv_nav4d::Motion::MOV_POINTTURN){
             continue;
        }
        firstPose = motion.intermediateStepsObstMap[0].pose;
        firstPose.position += startPosWorld.head<2>();
        posesOnPath.push_back(firstPose);

        intermediateStepsOk = true;
        for(size_t j = 1; j < motion.intermediateStepsObstMap.size(); ++j)
        {
            const PoseWithCell& pwc = motion.intermediateStepsObstMap[j];
            //diff is always a full offset to the start position
            const maps::grid::Index newIndex =  startIdxTravMap + pwc.cell;
            currentNode = currentNode->getConnectedNode(newIndex);
            if(currentNode == nullptr)
            {
                intermediateStepsOk = false;
                break;
            }
            nodesOnPath.push_back(currentNode);

            base::Pose2D curPose = pwc.pose;
            curPose.position += startPosWorld.head<2>();
            posesOnPath.push_back(curPose);

        }

        if(!intermediateStepsOk)
        {
            continue;
        }


        //check if the endpose is outside an obstacle
        std::vector<const traversability_generator3d::TravGenNode*> endPosePath;
        std::vector<base::Pose2D> endPosePoses;
        endPosePath.push_back(currentNode);
        Eigen::Vector3d endPosWorld;
        travMap->fromGrid(currentNode->getIndex(), endPosWorld, currentNode->getHeight(), false);
        base::Pose2D endPose;
        endPose.position = endPosWorld.topRows(2);
        endPose.orientation = motions[i].endTheta.getRadian();
        endPosePoses.push_back(endPose);
        PathStatistic endPoseStats(travConf);
        endPoseStats.calculateStatistics(endPosePath, endPosePoses, *travMap);
        if(endPoseStats.getRobotStats().getNumObstacles() > 0 ||
           endPoseStats.getRobotStats().getNumFrontiers() > 0)
        {
            //this path ends in an obstacle
            continue;
        }


        PathStatistic stats(travConf);
        stats.calculateStatistics(nodesOnPath, posesOnPath, *travMap);
        const int obstacleCount = stats.getRobotStats().getNumObstacles() + stats.getRobotStats().getNumFrontiers();

        if(obstacleCount < bestMotionObstacleCount)
        {
            bestMotionObstacleCount = obstacleCount;
            bestMotionIndex = i;
            bestNodesOnPath = nodesOnPath;
            bestPosesOnObstPath = posesOnPath;
        }
    }

    base::Trajectory trajectory;

    if(bestMotionIndex != -1)
    {
        //turn the poses into a spline
        std::vector<base::Vector3d> positions;
        Eigen::Hyperplane<double, 3> travNodePlane;

        assert(bestPosesOnObstPath.size() == bestNodesOnPath.size());
        
        for(size_t i = 0; i < bestPosesOnObstPath.size(); i++)
        {
            const traversability_generator3d::TravGenNode* curNode(bestNodesOnPath[i]);
            const base::Pose2D curPose(bestPosesOnObstPath[i]);

            Eigen::Vector3d posWorld;
            travMap->fromGrid(curNode->getIndex(), posWorld, curNode->getHeight(), false);

            // Set up the plane at the 3D world position
            travNodePlane.normal() = curNode->getUserData().plane.normal();
            travNodePlane.offset() = -travNodePlane.normal().dot(posWorld); // Align the plane offset to posWorld

            Eigen::Vector3d globalPoint{curPose.position.x(), curPose.position.y(), 0};
            Eigen::ParametrizedLine<double, 3> line = Eigen::ParametrizedLine<double, 3>::Through(globalPoint, globalPoint + Eigen::Vector3d::UnitZ());
            Eigen::Vector3d pointOnTravPlane = line.intersectionPoint(travNodePlane); 

            //TODO: Only left here until software which still uses trajectory2D is updated to use trajectory3D
            if (setZToZero){
                pointOnTravPlane.z() = 0;
            }

            Eigen::Vector3d pointOnBody = ground2Body.inverse(Eigen::Isometry) * pointOnTravPlane;
            if (positions.empty() || !(positions.back().isApprox(pointOnBody)))
            {
                positions.emplace_back(pointOnBody);
            }
        }

        trajectory.spline.interpolate(positions);
        trajectory.speed = motions[bestMotionIndex].type == Motion::Type::MOV_BACKWARD? -mobilityConfig.translationSpeed : mobilityConfig.translationSpeed;
#ifdef ENABLE_DEBUG_DRAWINGS
            V3DD::COMPLEX_DRAWING([&]()
            {
                for(base::Vector3d pos : positions)
                {
    //                 pos = travMap->getLocalFrame().inverse(Eigen::Isometry) * pos;
                    V3DD::DRAW_CYLINDER("ugv_nav4d_outOfObstacleTrajectory", pos,  base::Vector3d(0.02, 0.02, 0.2), V3DD::Color::blue);
                }
            });
#endif
    }
    else
    {
        LOG_ERROR_S<< "EnvironmentXYZTheta::findTrajectoryOutOfObstacle(): NO WAY OUT, ROBOT IS STUCK!";
        LOG_ERROR_S<< "EnvironmentXYZTheta::findTrajectoryOutOfObstacle(): NO WAY OUT, ROBOT IS STUCK!";
        LOG_ERROR_S<< "EnvironmentXYZTheta::findTrajectoryOutOfObstacle(): NO WAY OUT, ROBOT IS STUCK!";
        return nullptr;
    }

    std::shared_ptr<SubTrajectory> subTraj(new SubTrajectory(trajectory));
    subTraj->kind = trajectory_follower::TRAJECTORY_KIND_RESCUE;
    return subTraj;
}
}
