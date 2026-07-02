#include "EnvironmentXYZTheta.hpp"
#include <sbpl/planners/planner.h>
#include <sbpl/utils/mdpconfig.h>
#include <base/Pose.hpp>
#include <base/Spline.hpp>
#include <fstream>
#include "PathStatistic.hpp"
#include "Dijkstra.hpp"
#include <limits>
#include <chrono>
#include <base-logging/Logging.hpp>
#include <omp.h>

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
    , corridorWidth(-1.0)
    , availableMotions(primitiveConfig, mobilityConfig)
    , startThetaNode(nullptr)
    , startXYZNode(nullptr)
    , goalThetaNode(nullptr)
    , goalXYZNode(nullptr)
    , travConf(travConf)
    , primitiveConfig(primitiveConfig)
    , mobilityConfig(mobilityConfig)
    , goalOrientationMargin(0.0)
    , goalDistanceMargin(0.0)
    , planningMaxTime(-1.0)
{
    numAngles = primitiveConfig.numAngles;
    searchGrid.setResolution(Eigen::Vector2d(travConf.gridResolution, travConf.gridResolution));
    searchGrid.extend(travMap->getNumCells());
    robotHalfSize << travConf.robotSizeX / 2, travConf.robotSizeY / 2, travConf.robotHeight/2;
    if(travMap)
    {
        availableMotions.computeMotions(travConf.gridResolution);
    }

    usePathStatistics = false;
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
    transitionCache.clear();
    planningMaxTime = -1.0;
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
        availableMotions.computeMotions(travConf.gridResolution);
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
        LOG_ERROR_S << "createNewStateFromPose: could not find matching trav node for " << name << " at " << pos.transpose() << std::endl;
        return nullptr;
    }

    //check if intitial patch is unknown
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

    // Find patch within reasonable range of goal height
    traversability_generator3d::TravGenNode *bestMatch = nullptr;
    double minDistance = std::numeric_limits<double>::max();

    //check if we got an existing node
    for(traversability_generator3d::TravGenNode *snode : trList)
    {
        const double searchHeight = snode->getHeight();

        // Perfect match within step height
        if((searchHeight - travConf.maxStepHeight) <= pos.z() && (searchHeight + travConf.maxStepHeight) >= pos.z())
        {
            return snode;
        }

        // Find closest patch within reasonable distance
        double distance = std::abs(searchHeight - pos.z());
        if(distance < (travConf.maxStepHeight * 1.5) && distance < minDistance)
        {
            minDistance = distance;
            bestMatch = snode;
        }
    }
    return bestMatch;
}

bool EnvironmentXYZTheta::obstacleCheck(const maps::grid::Vector3d& pos, double theta,
                                        const traversability_generator3d::TraversabilityConfig& travConf,
                                        const SplinePrimitivesConfig& splineConf,
                                        const std::string& nodeName)
{
    traversability_generator3d::TravGenNode* travNode = findMatchingTraversabilityPatchAt(pos);
    if(!travNode)
    {
        LOG_ERROR_S << "obstacleCheck: could not find matching trav node for " << nodeName << " at " << pos.transpose() << std::endl;
        return false;
    }

    if (travNode->getUserData().nodeType == ::traversability_generator3d::NodeType::PARTIALLY_TRAVERSABLE)
    {
        if (!checkOrientationAllowed(travNode, theta))
        {
            return false;
        }
    }
    else if (travNode->getUserData().nodeType != ::traversability_generator3d::NodeType::TRAVERSABLE)
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
    travMap->fromGrid(node->getIndex(), nodePos, node->getHeight(), true);
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
    if(nodeType != ::traversability_generator3d::NodeType::TRAVERSABLE &&
       nodeType != ::traversability_generator3d::NodeType::PARTIALLY_TRAVERSABLE) {
        throw std::runtime_error("Error, goal has to be a traversable/partially-traversable patch");
    }


    if(travConf.enableInclineLimitting || nodeType == ::traversability_generator3d::NodeType::PARTIALLY_TRAVERSABLE)
    {
        if(!checkOrientationAllowed(goalXYZNode->getUserData().travNode, theta))
        {
            throw OrientationNotAllowed("Goal orientation not allowed");
        }
    }


    //NOTE If we want to precompute the heuristic (precomputeCost()) we need to expand
    //     the whole travmap beforehand.

    //check goal position
    if(!checkStartGoalNode("goal", goalXYZNode->getUserData().travNode, goalThetaNode->theta.getRadian()))
    {
        // Footprint collision is orientation-dependent, so this may be a heading issue rather
        // than the position itself being inside an obstacle.
        throw ObstacleCheckFailed("goal footprint in collision (position or orientation)");
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
            travMap->fromGrid(nextNode->getIndex(), pos, nextNode->getHeight(), true);

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
    traversability_generator3d::TravGenNode* startTravNode = startXYZNode->getUserData().travNode;
    const double startThetaRad = startThetaNode->theta.getRadian();

    // On a partially traversable cell a failure can be purely a disallowed orientation rather
    // than the position being inside an obstacle. Report that case distinctly so we only say
    // "inside obstacle" when the position is really non-traversable.
    if(startTravNode &&
       startTravNode->getUserData().nodeType == ::traversability_generator3d::NodeType::PARTIALLY_TRAVERSABLE &&
       !checkOrientationAllowed(startTravNode, startThetaRad))
    {
        LOG_ERROR_S << "Start orientation not allowed on partially traversable cell";
        throw ObstacleCheckFailed("Start orientation not allowed on partially traversable cell");
    }

    if(!checkStartGoalNode("start", startTravNode, startThetaRad))
    {
        LOG_ERROR_S << "Start position inside obstacle";
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
    uint64_t key = ((uint64_t)fromStateID << 32) | toStateID;
    auto it = transitionCache.find(key);
    if (it != transitionCache.end())
    {
        return availableMotions.getMotion(it->second.motionId);
    }

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

    if(travNode->getUserData().nodeType != ::traversability_generator3d::NodeType::TRAVERSABLE &&
       travNode->getUserData().nodeType != ::traversability_generator3d::NodeType::PARTIALLY_TRAVERSABLE)
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
    if(targetNode->getUserData().nodeType != ::traversability_generator3d::NodeType::TRAVERSABLE &&
       targetNode->getUserData().nodeType != ::traversability_generator3d::NodeType::PARTIALLY_TRAVERSABLE)
    {
        LOG_DEBUG_S<< "movement not possible. targetnode not traversable/partially-traversable";
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
    if (planningMaxTime > 0.0)
    {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - planningStartTime).count();
        if (elapsed > planningMaxTime)
        {
            LOG_DEBUG_S << "Planning timeout reached during GetSuccs: " << elapsed << "s / " << planningMaxTime << "s";
            SuccIDV->clear();
            CostV->clear();
            motionIdV.clear();
            return;
        }
    }

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
            Eigen::Vector3d pos;
            travMap->fromGrid(node->getIndex(), pos, node->getHeight(), true);
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
    travMap->fromGrid(sourceNode->getIndex(), sourcePosWorld, sourceTravNode->getHeight(), true);

    const auto& motions = availableMotions.getMotionForStartTheta(sourceThetaNode->theta);

    struct SuccessorCandidate
    {
        traversability_generator3d::TravGenNode* goalTravNode;
        maps::grid::Index finalPos;
        DiscreteTheta endTheta;
        int cost;
        size_t motionId;
        bool isPartiallyTraversable;

        SuccessorCandidate(traversability_generator3d::TravGenNode* goal,
                           const maps::grid::Index& pos,
                           const DiscreteTheta& theta,
                           int c,
                           size_t mId,
                           bool partially)
            : goalTravNode(goal), finalPos(pos), endTheta(theta), cost(c), motionId(mId), isPartiallyTraversable(partially) {}
    };

    int maxThreads = omp_get_max_threads();
    std::vector<std::vector<SuccessorCandidate>> threadCandidates(maxThreads);
    std::vector<std::vector<const traversability_generator3d::TravGenNode*>> threadNodes(maxThreads);
    std::vector<std::vector<base::Pose2D>> threadPoses(maxThreads);
    for(int t = 0; t < maxThreads; ++t)
    {
        threadNodes[t].reserve(32);
        threadPoses[t].reserve(32);
    }

    //dynamic scheduling is choosen because the iterations have vastly different runtime
    //due to the different sanity checks
    //the chunk size (5) was chosen to reduce dynamic scheduling overhead.
    //**No** tests have been done to verify whether 5 is a good value or not!
    #pragma omp parallel for schedule(dynamic, 5)
    for(size_t i = 0; i < motions.size(); ++i)
    {
        int threadId = omp_get_thread_num();
        const ugv_nav4d::Motion &motion(motions[i]);

        auto &nodesOnTravPath = threadNodes[threadId];
        auto &posesOnPath = threadPoses[threadId];
        nodesOnTravPath.clear();
        posesOnPath.clear();
        maps::grid::Index curIdx = sourceTravNode->getIndex();
        traversability_generator3d::TravGenNode *travNode = sourceTravNode;
        bool intermediateStepsOk = true;
        bool isPartiallyTraversable = false;
        int nodeBaseCost = 0;
        for(const PoseWithCell &diff : motion.intermediateStepsTravMap)
        {
            //diff is always a full offset to the start position
            const maps::grid::Index newIndex =  sourceTravNode->getIndex() + diff.cell;
            travNode = movementPossible(travNode, curIdx, newIndex);
            if(!travNode)
            {
                intermediateStepsOk = false;
                break;
            }
            if (corridorWidth > 0.0)
            {
                size_t nodeId = travNode->getUserData().id;
                if (nodeId >= nodeInCorridor.size() || !nodeInCorridor[nodeId])
                {
                    intermediateStepsOk = false;
                    break;
                }
            }
            nodesOnTravPath.push_back(travNode);

            nodeBaseCost += travNode->getUserData().cost;

            base::Pose2D curPose = diff.pose;
            curPose.position += sourcePosWorld.head<2>();
            posesOnPath.push_back(curPose);

            if(travNode->getUserData().nodeType == ::traversability_generator3d::NodeType::PARTIALLY_TRAVERSABLE)
            {
                if(!checkOrientationAllowed(travNode, diff.pose.orientation))
                {
                    intermediateStepsOk = false;
                    break;
                }
                isPartiallyTraversable = true;
            }
            else if(travConf.enableInclineLimitting)
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

        traversability_generator3d::TravGenNode *goalTravNode = travNode;

        if (corridorWidth > 0.0)
        {
            size_t succNodeId = goalTravNode->getUserData().id;
            if (succNodeId >= nodeInCorridor.size() || !nodeInCorridor[succNodeId])
                continue;
        }

        if (usePathStatistics){
            PathStatistic statistic(travConf);

            if(!statistic.isPathFeasible(nodesOnTravPath, posesOnPath, *getTraversabilityMap()))
            {
                continue;
            }
        }

        //goal from source to the end of the motion was valid
        const maps::grid::Index finalPos(sourceNode->getIndex() + maps::grid::Index(motion.xDiff,motion.yDiff));

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
                const double heightDiff = std::abs(sourceNode->getHeight() - goalTravNode->getHeight());
                //not perfect but probably more exact than the slope factors above
                const double approxMotionLen3D = std::sqrt(std::pow(motion.translationlDist, 2) + std::pow(heightDiff, 2));
                assert(approxMotionLen3D >= motion.translationlDist);//due to triangle inequality
                const double translationalVelocity = mobilityConfig.translationSpeed;
                cost = Motion::calculateCost(approxMotionLen3D, motion.angularDist, translationalVelocity,
                                             mobilityConfig.rotationSpeed, motion.costMultiplier, mobilityConfig.angularCostWeight);
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

        if (isPartiallyTraversable)
        {
            cost *= travConf.partiallyTraversableMultiplier;
        }

        cost += nodeBaseCost;

        oassert(cost <= std::numeric_limits<int>::max() && cost >= std::numeric_limits< int >::min());
        oassert(int(cost) >= motion.baseCost);
        oassert(motion.baseCost > 0);

        const int iCost = (int)cost;

        SuccessorCandidate cand(goalTravNode, finalPos, motion.endTheta, iCost, motion.id, isPartiallyTraversable);
        threadCandidates[threadId].push_back(cand);
    }

    // Process all gathered successor candidates sequentially to avoid locks and context-switching
    for (int t = 0; t < maxThreads; ++t)
    {
        for (const auto& cand : threadCandidates[t])
        {
            XYZNode *successXYNode = nullptr;
            ThetaNode *successthetaNode = nullptr;

            const auto &candidateMap = searchGrid.at(cand.finalPos);

            if(cand.goalTravNode->getIndex() != cand.finalPos){
                LOG_ERROR_S << "Internal error, indexes of goalTravNode and finalPos do not match";
                throw std::runtime_error("Internal error, indexes of goalTravNode and finalPos do not match");
            }
            XYZNode searchTmp(cand.goalTravNode->getHeight(), cand.goalTravNode->getIndex());

            auto it = candidateMap.find(&searchTmp);

            if(it != candidateMap.end())
            {
                successXYNode = *it;
            }
            else
            {
                successXYNode = createNewXYZState(cand.goalTravNode);
            }

            const auto &thetaMap(successXYNode->getUserData().thetaToNodes);

            bool isGoal = false;
            if (goalXYZNode && goalThetaNode)
            {
                bool withinDistance = (successXYNode == goalXYZNode);
                if (!withinDistance && goalDistanceMargin > 0.0)
                {
                    Eigen::Vector3d succPos, goalPos;
                    travMap->fromGrid(successXYNode->getIndex(), succPos, successXYNode->getHeight(), true);
                    travMap->fromGrid(goalXYZNode->getIndex(), goalPos, goalXYZNode->getHeight(), true);
                    if ((succPos.head<2>() - goalPos.head<2>()).norm() <= goalDistanceMargin)
                    {
                        withinDistance = true;
                    }
                }

                if (withinDistance)
                {
                    bool withinOrientation = false;
                    if (goalOrientationMargin > 0.0)
                    {
                        withinOrientation = (cand.endTheta.shortestDist(goalThetaNode->theta).getRadian() <= goalOrientationMargin);
                    }
                    else
                    {
                        withinOrientation = (cand.endTheta == goalThetaNode->theta);
                    }

                    if (withinOrientation)
                    {
                        isGoal = true;
                    }
                }
            }

            if (isGoal)
            {
                successthetaNode = goalThetaNode;
            }
            else
            {
                auto thetaCandidate = thetaMap.find(cand.endTheta);
                if(thetaCandidate != thetaMap.end())
                {
                    successthetaNode = thetaCandidate->second;
                }
                else
                {
                    successthetaNode = createNewState(cand.endTheta, successXYNode);
                }
            }

            SuccIDV->push_back(successthetaNode->id);
            CostV->push_back(cand.cost);
            motionIdV.push_back(cand.motionId);
            // Only cache the min-cost motion for each (from, to) pair so that
            // getMotion() returns the same motion the planner chose.
            uint64_t cacheKey = ((uint64_t)SourceStateID << 32) | successthetaNode->id;
            auto cacheIt = transitionCache.find(cacheKey);
            if (cacheIt == transitionCache.end() || cand.cost < cacheIt->second.cost)
            {
                transitionCache[cacheKey] = {cand.motionId, cand.cost};
            }

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
            travMap->fromGrid(curNode->getIndex(), posWorld, curNode->getHeight(), true);

            // Set up the plane at the 3D world position
            travNodePlane.normal() = curNode->getUserData().plane.normal();
            travNodePlane.offset() = -travNodePlane.normal().dot(posWorld); // Align the plane offset to posWorld

            for (const base::Pose2D &p : cwp.poses)
            {
                Eigen::Vector3d point{p.position.x(), p.position.y(), 0};
                Eigen::Vector3d globalPoint = point + start;
                Eigen::ParametrizedLine<double, 3> line = Eigen::ParametrizedLine<double, 3>::Through(globalPoint, globalPoint + Eigen::Vector3d::UnitZ());
                Eigen::Vector3d pointOnTravPlane;
                // If the plane normal is nearly vertical (Z component ≈ 0), the vertical
                // projection line is parallel to the plane → intersection is undefined → NaN.
                // Fall back to using the node's world height directly.
                if (std::abs(travNodePlane.normal().z()) < 1e-6)
                {
                    pointOnTravPlane = globalPoint;
                    pointOnTravPlane.z() = posWorld.z();
                }
                else
                {
                    pointOnTravPlane = line.intersectionPoint(travNodePlane);
                }
#ifdef ENABLE_DEBUG_DRAWINGS
                V3DD::DRAW_SPHERE("ugv_nav4d_trajectory_poses", pointOnTravPlane, 0.01, V3DD::Color::red);
#endif
                //TODO: Only left here until software which still uses trajectory2D is updated to use trajectory3D
                if (setZToZero){
                    pointOnTravPlane.z() = 0;
                }

                Eigen::Vector3d pointOnBody = plan2Body.inverse(Eigen::Isometry) * pointOnTravPlane;
                if (positions.empty() || (positions.back() - pointOnBody).norm() > 1e-3)
                {
                    positions.emplace_back(pointOnBody);
                }
            }
        }
        if (mobilityConfig.remove_goal_offset == true &&
            i == indexOfMotionToUpdate)
        {
            if (positions.size() >= 2)
            {
                double goal_offset_x = (goalPos.x() - positions[positions.size()-1].x()) / (positions.size()-1);
                double goal_offset_y = (goalPos.y() - positions[positions.size()-1].y()) / (positions.size()-1);

                for (std::size_t j{0}; j < positions.size(); j++){
                    positions[j].x() += j*goal_offset_x;
                    positions[j].y() += j*goal_offset_y;
                }
            }
            else if (positions.size() == 1)
            {
                positions[0] = goalPos;
            }
            updateGoalPose = true;
        }

        // Smooth Z to remove height discontinuities at cell boundaries.
        // Uses a 3-point weighted average (0.25, 0.5, 0.25) while preserving
        // the start and end heights to maintain segment continuity.
        if (!setZToZero && positions.size() >= 3)
        {
            std::vector<double> smoothedZ(positions.size());
            smoothedZ[0] = positions[0].z();
            smoothedZ.back() = positions.back().z();
            for (size_t k = 1; k < positions.size() - 1; ++k)
            {
                smoothedZ[k] = 0.25 * positions[k-1].z() + 0.5 * positions[k].z() + 0.25 * positions[k+1].z();
            }
            for (size_t k = 0; k < positions.size(); ++k)
            {
                positions[k].z() = smoothedZ[k];
            }
        }


        if (curMotion.type != Motion::Type::MOV_POINTTURN)
        {
            if (positions.size() >= 2){
                curPart.spline.interpolate(positions);
            }
            else if (positions.size() == 1){
                curPart.spline.setSingleton(positions[0]);
            }
        }

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
            SubTrajectory curPartSub;
            curPartSub.driveMode = DriveMode::ModeTurnOnTheSpot;

            curPartSub.posSpline.setSingleton(start);

            std::vector<double> anglesd;
            anglesd.emplace_back(curMotion.startTheta.getRadian());
            anglesd.emplace_back(curMotion.endTheta.getRadian());
            curPartSub.orientationSpline.interpolate(anglesd);

            base::Pose2D startPose;
            startPose.position.x() = start.x();
            startPose.position.y() = start.y();
            startPose.orientation  = curMotion.startTheta.getRadian();
            curPartSub.startPose     = startPose;

            base::Pose2D goalPose;
            goalPose.position.x() = start.x();
            goalPose.position.y() = start.y();
            goalPose.orientation  = curMotion.endTheta.getRadian();
            curPartSub.goalPose      = goalPose;

            result.push_back(curPartSub);
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

double EnvironmentXYZTheta::getAvgSlope(const std::vector<const traversability_generator3d::TravGenNode*>& path) const
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

double EnvironmentXYZTheta::getMaxSlope(const std::vector<const traversability_generator3d::TravGenNode*>& path) const
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
    auto start_time = std::chrono::steady_clock::now();
    std::unordered_map<const maps::grid::TraversabilityNodeBase*, double> costToStart;
    std::unordered_map<const maps::grid::TraversabilityNodeBase*, double> costToEnd;

    // Compute costs in parallel
    #pragma omp parallel sections
    {
        #pragma omp section
        {
            Dijkstra::computeCost(startXYZNode->getUserData().travNode, costToStart, travConf, mobilityConfig);
        }
        #pragma omp section
        {
            Dijkstra::computeCost(goalXYZNode->getUserData().travNode, costToEnd, travConf, mobilityConfig);
        }
    }
    auto after_dijkstra_end = std::chrono::steady_clock::now();
    auto after_dijkstra_start = after_dijkstra_end;

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
    travNodeIdToDistance.resize(largestId + 1, Distance(maxDist, maxDist));

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

    // Compute the corridor mask
    if (corridorWidth > 0.0)
    {
        nodeInCorridor.assign(largestId + 1, false);

        std::vector<traversability_generator3d::TravGenNode*> greedyPath;
        traversability_generator3d::TravGenNode* nextNode = startXYZNode->getUserData().travNode;
        traversability_generator3d::TravGenNode* goal = goalXYZNode->getUserData().travNode;

        greedyPath.push_back(nextNode);

        bool reachedGoal = (nextNode == goal);
        while(nextNode != goal)
        {
            double minCost = std::numeric_limits<double>::max();
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
                break;
            }
            greedyPath.push_back(nextNode);

            if (nextNode == goal)
            {
                reachedGoal = true;
            }
        }

        if (reachedGoal)
        {
            std::vector<double> allowedWidths(greedyPath.size(), corridorWidth);
            for(size_t i = 1; i + 1 < greedyPath.size(); ++i)
            {
                Eigen::Vector3d posPrev, posCurr, posNext;
                travMap->fromGrid(greedyPath[i-1]->getIndex(), posPrev, greedyPath[i-1]->getHeight(), true);
                travMap->fromGrid(greedyPath[i]->getIndex(), posCurr, greedyPath[i]->getHeight(), true);
                travMap->fromGrid(greedyPath[i+1]->getIndex(), posNext, greedyPath[i+1]->getHeight(), true);

                Eigen::Vector2d v1 = (posCurr - posPrev).head<2>();
                Eigen::Vector2d v2 = (posNext - posCurr).head<2>();
                double n1 = v1.norm();
                double n2 = v2.norm();
                if (n1 > 1e-5 && n2 > 1e-5)
                {
                    v1 /= n1;
                    v2 /= n2;
                    double dot = v1.dot(v2);
                    dot = std::max(-1.0, std::min(1.0, dot));
                    double angleDiff = std::acos(dot);
                    
                    double turnFactor = std::min(1.0, angleDiff / (M_PI / 2.0));
                    allowedWidths[i] = corridorWidth + turnFactor * mobilityConfig.minTurningRadius;
                }
            }

            struct QueueElement
            {
                traversability_generator3d::TravGenNode* node;
                double dist;
                double allowedWidth;
            };
            std::queue<QueueElement> bfsQueue;
            std::vector<maps::grid::Vector3d> corridorPositions;

            for (size_t i = 0; i < greedyPath.size(); ++i)
            {
                nodeInCorridor[greedyPath[i]->getUserData().id] = true;
                bfsQueue.push({greedyPath[i], 0.0, allowedWidths[i]});

                maps::grid::Vector3d p;
                travMap->fromGrid(greedyPath[i]->getIndex(), p, greedyPath[i]->getHeight(), true);
                corridorPositions.push_back(p);
            }

            const double res = travConf.gridResolution;
            while(!bfsQueue.empty())
            {
                auto current = bfsQueue.front();
                bfsQueue.pop();

                traversability_generator3d::TravGenNode* u = current.node;
                double dist = current.dist;
                double allowedWidth = current.allowedWidth;

                if (dist >= allowedWidth)
                    continue;

                for(maps::grid::TraversabilityNodeBase* node : u->getConnections())
                {
                    if (node->getType() != maps::grid::TraversabilityNodeBase::TRAVERSABLE)
                        continue;
                    traversability_generator3d::TravGenNode* v = static_cast<traversability_generator3d::TravGenNode*>(node);
                    size_t vId = v->getUserData().id;
                    if (!nodeInCorridor[vId])
                    {
                        nodeInCorridor[vId] = true;
                        bfsQueue.push({v, dist + res, allowedWidth});

                        maps::grid::Vector3d p;
                        travMap->fromGrid(v->getIndex(), p, v->getHeight(), true);
                        corridorPositions.push_back(p);
                    }
                }
            }

#ifdef ENABLE_DEBUG_DRAWINGS
            V3DD::COMPLEX_DRAWING([&]()
            {
                V3DD::CLEAR_DRAWING("ugv_nav4d_corridor");
                for (const auto& pos : corridorPositions)
                {
                    V3DD::DRAW_SPHERE("ugv_nav4d_corridor", pos, 0.08, V3DD::Color::green);
                }
            });
#endif
        }
        else
        {
            LOG_WARN_S << "Greedy path did not reach goal. Disabling corridor pruning.";
            nodeInCorridor.assign(largestId + 1, true);
#ifdef ENABLE_DEBUG_DRAWINGS
            V3DD::COMPLEX_DRAWING([&]()
            {
                V3DD::CLEAR_DRAWING("ugv_nav4d_corridor");
            });
#endif
        }
    }
    else
    {
        nodeInCorridor.assign(largestId + 1, true);
#ifdef ENABLE_DEBUG_DRAWINGS
        V3DD::COMPLEX_DRAWING([&]()
        {
            V3DD::CLEAR_DRAWING("ugv_nav4d_corridor");
        });
#endif
    }

    auto end_time = std::chrono::steady_clock::now();
    double t_dijkstra_start = std::chrono::duration<double>(after_dijkstra_start - start_time).count();
    double t_dijkstra_end = std::chrono::duration<double>(after_dijkstra_end - after_dijkstra_start).count();
    double t_mapping = std::chrono::duration<double>(end_time - after_dijkstra_end).count();
    LOG_INFO_S << "[KPI] precomputeCost - Dijkstra Start: " << t_dijkstra_start << "s, Dijkstra End: " << t_dijkstra_end << "s, Distance Mapping: " << t_mapping << "s, Total: " << std::chrono::duration<double>(end_time - start_time).count() << "s";
}

void EnvironmentXYZTheta::setCorridorWidth(double width)
{
    corridorWidth = width;
}

void EnvironmentXYZTheta::setGoalOrientationMargin(double margin)
{
    goalOrientationMargin = margin;
}

void EnvironmentXYZTheta::setGoalDistanceMargin(double margin)
{
    goalDistanceMargin = margin;
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
    travMap->fromGrid(startTravNode->getIndex(), startPosWorld, startTravNode->getHeight(), true);

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
        //NOTE: Pointturns have no intermediateStepsTravMap, so are skipped at the moment.
        if (motion.type == ugv_nav4d::Motion::MOV_POINTTURN){
             continue;
        }
        firstPose = motion.intermediateStepsTravMap[0].pose;
        firstPose.position += startPosWorld.head<2>();
        posesOnPath.push_back(firstPose);

        intermediateStepsOk = true;
        for(size_t j = 1; j < motion.intermediateStepsTravMap.size(); ++j)
        {
            const PoseWithCell& pwc = motion.intermediateStepsTravMap[j];
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
        travMap->fromGrid(currentNode->getIndex(), endPosWorld, currentNode->getHeight(), true);
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
            travMap->fromGrid(curNode->getIndex(), posWorld, curNode->getHeight(), true);

            // Set up the plane at the 3D world position
            travNodePlane.normal() = curNode->getUserData().plane.normal();
            travNodePlane.offset() = -travNodePlane.normal().dot(posWorld); // Align the plane offset to posWorld

            Eigen::Vector3d globalPoint{curPose.position.x(), curPose.position.y(), 0};
            Eigen::ParametrizedLine<double, 3> line = Eigen::ParametrizedLine<double, 3>::Through(globalPoint, globalPoint + Eigen::Vector3d::UnitZ());
            Eigen::Vector3d pointOnTravPlane;
            if (std::abs(travNodePlane.normal().z()) < 1e-6)
            {
                pointOnTravPlane = globalPoint;
                pointOnTravPlane.z() = posWorld.z();
            }
            else
            {
                pointOnTravPlane = line.intersectionPoint(travNodePlane);
            }

            //TODO: Only left here until software which still uses trajectory2D is updated to use trajectory3D
            if (setZToZero){
                pointOnTravPlane.z() = 0;
            }

            Eigen::Vector3d pointOnBody = ground2Body.inverse(Eigen::Isometry) * pointOnTravPlane;
            if (positions.empty() || (positions.back() - pointOnBody).norm() > 1e-3)
            {
                positions.emplace_back(pointOnBody);
            }
        }

        if (positions.size() >= 2)
        {
            trajectory.spline.interpolate(positions);
        }
        else if (positions.size() == 1)
        {
            trajectory.spline.setSingleton(positions[0]);
        }
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
