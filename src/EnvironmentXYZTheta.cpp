#include "EnvironmentXYZTheta.hpp"
#include <sbpl/planners/planner.h>
#include <sbpl/utils/mdpconfig.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <base/Pose.hpp>
#include <base/Spline.hpp>
#include <fstream>
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>
#include <vizkit3d_debug_drawings/DebugDrawingColors.hpp>
#include "PathStatistic.hpp"
#include "Dijkstra.hpp"
#include <trajectory_follower/SubTrajectory.hpp>
#include <limits>
#include <trajectory_follower/SubTrajectory.hpp>
#include <base-logging/Logging.hpp>

using namespace std;
using namespace sbpl_spline_primitives;
using trajectory_follower::SubTrajectory;
using trajectory_follower::DriveMode;

//#define ENABLE_V3DD_DRAWINGS

namespace ugv_nav4d
{

#define oassert(val) \
    if(!(val)) \
    {\
        LOG_ERROR_S << #val; \
        LOG_ERROR_S << __FILE__ << ": " << __LINE__; \
        throw std::runtime_error("meeeeh"); \
    }

EnvironmentXYZTheta::EnvironmentXYZTheta(std::shared_ptr<MLGrid> mlsGrid,
                                         const traversability_generator3d::TraversabilityConfig& travConf,
                                         const SplinePrimitivesConfig& primitiveConfig,
                                         const Mobility& mobilityConfig) :
    travGen(travConf), obsGen(travConf)
    , mlsGrid(mlsGrid)
    , availableMotions(primitiveConfig, mobilityConfig)
    , startThetaNode(nullptr)
    , startXYZNode(nullptr)
    , goalThetaNode(nullptr)
    , goalXYZNode(nullptr)
    , obstacleStartNode(nullptr)
    , travConf(travConf)
    , primitiveConfig(primitiveConfig)
    , mobilityConfig(mobilityConfig)
{
    numAngles = primitiveConfig.numAngles;
    travGen.setMLSGrid(mlsGrid);
    obsGen.setMLSGrid(mlsGrid);
    searchGrid.setResolution(Eigen::Vector2d(travConf.gridResolution, travConf.gridResolution));
    searchGrid.extend(travGen.getTraversabilityMap().getNumCells());
    robotHalfSize << travConf.robotSizeX / 2, travConf.robotSizeY / 2, travConf.robotHeight/2;
    if(mlsGrid)
    {
        availableMotions.computeMotions(mlsGrid->getResolution().x(), travConf.gridResolution);
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

void EnvironmentXYZTheta::setInitialPatch(const Eigen::Affine3d& ground2Mls, double patchRadius)
{
    travGen.setInitialPatch(ground2Mls, patchRadius);
    obsGen.setInitialPatch(ground2Mls, patchRadius);
}

void EnvironmentXYZTheta::updateMap(shared_ptr< ugv_nav4d::EnvironmentXYZTheta::MLGrid > mlsGrid)
{
    if(this->mlsGrid && this->mlsGrid->getResolution() != mlsGrid->getResolution())
        throw std::runtime_error("EnvironmentXYZTheta::updateMap : Error got MLSMap with different resolution");

    if(!this->mlsGrid)
    {
        availableMotions.computeMotions(mlsGrid->getResolution().x(), travConf.gridResolution);
    }
    travGen.setMLSGrid(mlsGrid);
    obsGen.setMLSGrid(mlsGrid);
    this->mlsGrid = mlsGrid;

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
    traversability_generator3d::TravGenNode *travNode = travGen.generateStartNode(pos);
    if(!travNode)
    {
        LOG_INFO_S << "Could not generate Node at pos";
        return nullptr;
    }

    //check if intitial patch is unknown
    if(!travNode->isExpanded())
    {
        if(!travGen.expandNode(travNode))
        {
            cout << "createNewStateFromPose: Error: " << name << " Pose " << pos.transpose() << " is not traversable" << endl;
            return nullptr;
        }
        travNode->setNotExpanded();
    }

    XYZNode *xyzNode = createNewXYZState(travNode);

    DiscreteTheta thetaD(theta, numAngles);

    if(xyzBackNode)
        *xyzBackNode = xyzNode;

    return createNewState(thetaD, xyzNode);
}

bool EnvironmentXYZTheta::obstacleCheck(const maps::grid::Vector3d& pos, double theta,
                                        const ObstacleMapGenerator3D& obsGen,
                                        const traversability_generator3d::TraversabilityConfig& travConf,
                                        const SplinePrimitivesConfig& splineConf,
                                        const std::string& nodeName)
{
    //PathStatistic stats(travConf);
    //std::vector<base::Pose2D> poses;

    maps::grid::Index idxObstNode;
    if(!obsGen.getTraversabilityMap().toGrid(pos, idxObstNode))
    {
        LOG_INFO_S << "Error " << nodeName << " is outside of obstacle map ";
        return false;
    }
    traversability_generator3d::TravGenNode* obstacleNode = obsGen.findMatchingTraversabilityPatchAt(idxObstNode, pos.z());
    if(!obstacleNode)
    {
        LOG_INFO_S << "Error, could not find matching obstacle node for " << nodeName;
        return false;
    }

    if (obstacleNode->getType() != ::maps::grid::TraversabilityNodeBase::TRAVERSABLE)
    {
        return false;
    }

    return true;

    /*
    //DISABLED STATS CALCULATED 26.08.2022 by modhi

    std::vector<const traversability_generator3d::TravGenNode*> path;
    path.push_back(obstacleNode);

    const Eigen::Vector3d centeredPos = obstacleNode->getPosition(obsGen.getTraversabilityMap());


    //NOTE theta needs to be discretized because the planner uses discrete theta internally everywhere.
    //     If we do not discretize here, external calls and internal calls will have different results for the same pose input

    DiscreteTheta discTheta(theta, splineConf.numAngles);

    poses.push_back(base::Pose2D(centeredPos.topRows(2), discTheta.getRadian()));

    stats.calculateStatistics(path, poses, obsGen.getTraversabilityMap(), "ugv_nav4d_" + nodeName + "Box");

    if(stats.getRobotStats().getNumObstacles() ) // || stats.getRobotStats().getNumFrontiers())  )
    {
#ifdef ENABLE_V3DD_DRAWINGS
        V3DD::COMPLEX_DRAWING([&]()
        {
            const std::string drawName("ugv_nav4d_obs_check_fail_" + nodeName);
            V3DD::CLEAR_DRAWING(drawName);
            V3DD::DRAW_WIREFRAME_BOX(drawName, pos, Eigen::Quaterniond(Eigen::AngleAxisd(discTheta.getRadian(), Eigen::Vector3d::UnitZ())), Eigen::Vector3d(travConf.robotSizeX, travConf.robotSizeY, travConf.robotHeight), V3DD::Color::red);
        });
#endif

        LOG_INFO_S << "Num obstacles: " << stats.getRobotStats().getNumObstacles();
        LOG_INFO_S << "Error: " << nodeName << " inside obstacle";
        return false;
    }


    return true;
    */
}

bool EnvironmentXYZTheta::checkStartGoalNode(const string& name, traversability_generator3d::TravGenNode *node, double theta)
{
    //check for collisions NOTE has to be done after expansion

    maps::grid::Vector3d nodePos;
    travGen.getTraversabilityMap().fromGrid(node->getIndex(), nodePos, node->getHeight(), false);
#ifdef ENABLE_V3DD_DRAWINGS
        V3DD::COMPLEX_DRAWING([&]()
        {
            const std::string drawName("ugv_nav4d_check_start_goal_" + name);
            V3DD::CLEAR_DRAWING(drawName);
            V3DD::DRAW_WIREFRAME_BOX(drawName, nodePos, Eigen::Quaterniond(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ())), Eigen::Vector3d(travConf.robotSizeX, travConf.robotSizeY, travConf.robotHeight), V3DD::Color::red);
        });
#endif


    return obstacleCheck(nodePos, theta, obsGen, travConf, primitiveConfig, name);
}


void EnvironmentXYZTheta::setGoal(const Eigen::Vector3d& goalPos, double theta)
{

#ifdef ENABLE_V3DD_DRAWINGS
    V3DD::CLEAR_DRAWING("ugv_nav4d_env_goalPos");
    V3DD::DRAW_ARROW("ugv_nav4d_env_goalPos", goalPos, base::Quaterniond(Eigen::AngleAxisd(M_PI, base::Vector3d::UnitX())),
            base::Vector3d(1,1,1), V3DD::Color::red);
#endif

    LOG_INFO_S << "GOAL IS: " << goalPos.transpose();

    if(!startXYZNode)
        throw std::runtime_error("Error, start needs to be set before goal");

    goalThetaNode = createNewStateFromPose("goal", goalPos, theta, &goalXYZNode);
    if(!goalThetaNode)
    {
        throw StateCreationFailed("Failed to create goal state");
    }
    const auto nodeType = goalXYZNode->getUserData().travNode->getType();
    if(nodeType != maps::grid::TraversabilityNodeBase::TRAVERSABLE) {
        throw std::runtime_error("Error, goal has to be a traversable patch");
    }


    if(travConf.enableInclineLimitting)
    {
        if(!checkOrientationAllowed(goalXYZNode->getUserData().travNode, theta))
        {
            LOG_INFO_S << "Goal orientation not allowed due to slope";
            throw OrientationNotAllowed("Goal orientation not allowed due to slope");
        }
    }


    //NOTE If we want to precompute the heuristic (precomputeCost()) we need to expand
    //     the whole travmap beforehand.

    //check goal position
    if(!checkStartGoalNode("goal", goalXYZNode->getUserData().travNode, goalThetaNode->theta.getRadian()))
    {
        LOG_INFO_S << "goal position is invalid";
        throw ObstacleCheckFailed("goal position is invalid");
    }

    precomputeCost();
    LOG_INFO_S << "Heuristic computed";
    //draw greedy path
#ifdef ENABLE_V3DD_DRAWINGS
    V3DD::COMPLEX_DRAWING([&]()
    {
        V3DD::CLEAR_DRAWING("ugv_nav4d_greedyPath");
        traversability_generator3d::TravGenNode* nextNode = startXYZNode->getUserData().travNode;
        traversability_generator3d::TravGenNode* goal = goalXYZNode->getUserData().travNode;
        while(nextNode != goal)
        {
            maps::grid::Vector3d pos;
            travGen.getTraversabilityMap().fromGrid(nextNode->getIndex(), pos, nextNode->getHeight(), false);

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
                LOG_INFO_S << "nextNode has no connection";
                break;
            }
        }
    });
#endif
}

void EnvironmentXYZTheta::expandMap(const std::vector<Eigen::Vector3d>& positions)
{
#ifdef ENABLE_V3DD_DRAWINGS
    V3DD::COMPLEX_DRAWING([&]()
    {
        V3DD::CLEAR_DRAWING("ugv_nav4d_expandStarts");
        for(const Eigen::Vector3d& pos : positions)
        {
            V3DD::DRAW_ARROW("ugv_nav4d_expandStarts", pos, base::Quaterniond(Eigen::AngleAxisd(M_PI, base::Vector3d::UnitX())),
                       base::Vector3d(1,1,1), V3DD::Color::cyan);
        }
    });
#endif

    travGen.expandAll(positions);
    obsGen.expandAll(positions);
}


void EnvironmentXYZTheta::setStart(const Eigen::Vector3d& startPos, double theta)
{
#ifdef ENABLE_V3DD_DRAWINGS
        V3DD::CLEAR_DRAWING("ugv_nav4d_env_startPos");
        V3DD::DRAW_ARROW("ugv_nav4d_env_startPos", startPos, base::Quaterniond(Eigen::AngleAxisd(M_PI, base::Vector3d::UnitX())),
                     base::Vector3d(1,1,1), V3DD::Color::blue);
#endif

    LOG_INFO_S << "START IS: " << startPos.transpose();

    startThetaNode = createNewStateFromPose("start", startPos, theta, &startXYZNode);
    if(!startThetaNode)
        throw StateCreationFailed("Failed to create start state");

    obstacleStartNode = obsGen.generateStartNode(startPos);
    if(!obstacleStartNode)
    {
        LOG_INFO_S << "Could not generate obstacle node at start pos";
        throw ObstacleCheckFailed("Could not generate obstacle node at start pos");
    }

    LOG_INFO_S<< "Expanding trav map...\n";
    travGen.expandAll(startXYZNode->getUserData().travNode);
    LOG_INFO_S<< "expanded ";

    LOG_INFO_S<< "Expanding obstacle map...\n";
    obsGen.expandAll(obstacleStartNode);
    LOG_INFO_S<< "expanded ";

    //check start position
    if(!checkStartGoalNode("start", startXYZNode->getUserData().travNode, startThetaNode->theta.getRadian()))
    {
        LOG_INFO_S<< "Start position is invalid";
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
    travGen.getTraversabilityMap().fromGrid(node->getIndex(), ret, node->getHeight());
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


int EnvironmentXYZTheta::GetGoalHeuristic(int stateID)
{

    // the heuristic distance has been calculated beforehand. Here it is just converted to
    // travel time.

    const Hash &sourceHash(idToHash[stateID]);
    const XYZNode *sourceNode = sourceHash.node;
    const traversability_generator3d::TravGenNode* travNode = sourceNode->getUserData().travNode;
    const ThetaNode *sourceThetaNode = sourceHash.thetaNode;

    if(travNode->getType() != maps::grid::TraversabilityNodeBase::TRAVERSABLE && travNode->getType() != maps::grid::TraversabilityNodeBase::FRONTIER)
    {
        std::map<int, std::string> numToTravType;
        numToTravType[maps::grid::TraversabilityNodeBase::OBSTACLE] = "OBSTACLE";
        numToTravType[maps::grid::TraversabilityNodeBase::TRAVERSABLE] = "TRAVERSABLE";
        numToTravType[maps::grid::TraversabilityNodeBase::UNKNOWN] = "UNKNOWN";
        numToTravType[maps::grid::TraversabilityNodeBase::HOLE] = "HOLE";
        numToTravType[maps::grid::TraversabilityNodeBase::UNSET] = "UNSET";
        numToTravType[maps::grid::TraversabilityNodeBase::FRONTIER] = "FRONTIER";
        //throw std::runtime_error("tried to get heuristic for " + numToTravType[travNode->getType()] + " patch. StateID: " + std::to_string(stateID));
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
        LOG_INFO_S << sourceToGoalDist;
        LOG_INFO_S << stateID;
        LOG_INFO_S << mobilityConfig.translationSpeed;
        LOG_INFO_S << timeTranslation;
        LOG_INFO_S << sourceThetaNode->theta.shortestDist(goalThetaNode->theta).getRadian();
        LOG_INFO_S << mobilityConfig.rotationSpeed;
        LOG_INFO_S << timeRotation;
        LOG_INFO_S << result;
        LOG_INFO_S << travNode->getUserData().id;
        LOG_INFO_S << travNode->getType();
        //throw std::runtime_error("Goal heuristic < 0");
        LOG_INFO_S<< "Overflow while computing goal heuristic!";
        result = std::numeric_limits<int>::max();
    }
    oassert(result >= 0);
    return result;
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
        LOG_INFO_S<< "movement not possible. nodes not conncted";
        return nullptr;
    }

    if(!checkExpandTreadSafe(targetNode))
    {
        return nullptr;
    }

    //NOTE this check cannot be done before checkExpandTreadSafe because the type will be determined
    //     during the expansion. Beforehand the type is undefined
    if(targetNode->getType() != maps::grid::TraversabilityNodeBase::TRAVERSABLE)
    {
//         LOG_INFO_S<< "movement not possible. targetnode not traversable";
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

    bool result = true;
    #pragma omp critical(checkExpandTreadSafe)
    {
        if(!node->isExpanded())
        {
            result = travGen.expandNode(node);
        }
    }
    return result;
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

#ifdef ENABLE_V3DD_DRAWINGS
        V3DD::COMPLEX_DRAWING([&]()
        {

            const traversability_generator3d::TravGenNode* node = sourceNode->getUserData().travNode;
            Eigen::Vector3d pos((node->getIndex().x() + 0.5) * travConf.gridResolution,
                                (node->getIndex().y() + 0.5) * travConf.gridResolution,
                                node->getHeight());
            pos = mlsGrid->getLocalFrame().inverse(Eigen::Isometry) * pos;
            V3DD::DRAW_WIREFRAME_BOX("ugv_nav4d_successors", pos, base::Vector3d(mlsGrid->getResolution().x() / 2.0, mlsGrid->getResolution().y() / 2.0,
                            0.05), V3DD::Color::blue);
        });
#endif

    if(!sourceTravNode->isExpanded())
    {
        if(!travGen.expandNode(sourceTravNode))
        {
            //expansion failed, current node is not driveable -> there are not successors to this state
            LOG_INFO_S<< "GetSuccs: current node not expanded and not expandable";
            return;
        }
    }

    Eigen::Vector3d sourcePosWorld;
    travGen.getTraversabilityMap().fromGrid(sourceNode->getIndex(), sourcePosWorld, sourceTravNode->getHeight(), false);

    traversability_generator3d::TravGenNode *sourceObstacleNode = findObstacleNode(sourceTravNode);
    assert(sourceObstacleNode);

    const auto& motions = availableMotions.getMotionForStartTheta(sourceThetaNode->theta);

    //dynamic scheduling is choosen because the iterations have vastly different runtime
    //due to the different sanity checks
    //the chunk size (5) was chosen to reduce dynamic scheduling overhead.
    //**No** tests have been done to verify whether 5 is a good value or not!
    //#pragma omp parallel for schedule(dynamic, 5)
    #pragma omp parallel for schedule(auto)
    for(size_t i = 0; i < motions.size(); ++i)
    {
        //check that the motion is traversable (without collision checks) and find the goal node of the motion
        const ugv_nav4d::Motion &motion(motions[i]);
        traversability_generator3d::TravGenNode *goalTravNode = checkTraversableHeuristic(sourceNode->getIndex(), sourceNode->getUserData().travNode, motions[i], travGen.getTraversabilityMap());
        if(!goalTravNode)
        {
            //at least one node on the path is not traversable
            continue;
        }

        //check motion path on obstacle map
        std::vector<const traversability_generator3d::TravGenNode*> nodesOnObstPath;
        std::vector<base::Pose2D> posesOnObstPath;
        maps::grid::Index curObstIdx = sourceObstacleNode->getIndex();
        traversability_generator3d::TravGenNode *obstNode = sourceObstacleNode;
        bool intermediateStepsOk = true;
        for(const PoseWithCell &diff : motion.intermediateStepsObstMap)
        {
            //diff is always a full offset to the start position
            const maps::grid::Index newIndex =  sourceObstacleNode->getIndex() + diff.cell;
            obstNode = movementPossible(obstNode, curObstIdx, newIndex);
            nodesOnObstPath.push_back(obstNode);
            base::Pose2D curPose = diff.pose;
            curPose.position += sourcePosWorld.head<2>();
            posesOnObstPath.push_back(curPose);
            if(!obstNode)
            {
                intermediateStepsOk = false;
                break;
            }

            if(travConf.enableInclineLimitting)
            {
                if(!checkOrientationAllowed(obstNode, diff.pose.orientation))
                {
                    intermediateStepsOk = false;
                    break;
                }
            }
            curObstIdx = newIndex;
        }


        //no way from start to end on obstacle map
        if(!intermediateStepsOk)
            continue;
        /*
        PathStatistic statistic(travConf);

        if(!statistic.isPathFeasible(nodesOnObstPath, posesOnObstPath, getObstacleMap()))
        {
            continue;
        }
        */

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

            if(goalTravNode->getIndex() != finalPos)
                throw std::runtime_error("Internal error, indexes do not match");

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
                if(nodesOnObstPath.size() > 0)
                {
                    avgSlope = getAvgSlope(nodesOnObstPath);
                }
                else
                {
                    //This happens on point turns as they have no intermediate steps
                    avgSlope = sourceTravNode->getUserData().slope;
                }
                const double slopeFactor = avgSlope * travConf.slopeMetricScale;
                cost = motion.baseCost + motion.baseCost * slopeFactor;
                LOG_INFO_S<< "cost: " << cost << ", baseCost: " << motion.baseCost << ", slopeFactor: " << slopeFactor;
                break;
            }
            case traversability_generator3d::SlopeMetric::MAX_SLOPE:
            {
                double maxSlope = 0;
                if(nodesOnObstPath.size() > 0)
                {
                    maxSlope = getMaxSlope(nodesOnObstPath);
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
                throw std::runtime_error("unknown slope metric selected");
        }

        /*
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
        */
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
                                        const Eigen::Vector3d &goalPos, double goalHeading, const Eigen::Affine3d &plan2Body)
{
    if(stateIDPath.size() < 2)
        return;

    result.clear();
    base::Trajectory curPart;

#ifdef ENABLE_V3DD_DRAWINGS
        V3DD::CLEAR_DRAWING("ugv_nav4d_trajectory");
#endif

    int indexOfMotionToUpdate{stateIDPath.size()-2};
    const Motion& finalMotion = getMotion(stateIDPath[stateIDPath.size()-2], stateIDPath[stateIDPath.size()-1]);
    if (finalMotion.type == Motion::Type::MOV_POINTTURN && stateIDPath.size() > 1){ //assuming that there are no consecutive point turns motion at the end of a planned trajectory
        indexOfMotionToUpdate = stateIDPath.size()-3;
    }

    bool goal_position_updated = false;

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
                    for(auto *n : curNode->getConnections())
                        LOG_INFO_S<< "Con Node " << n->getIndex().transpose();;
                    throw std::runtime_error("Internal error, trajectory is not continuous on tr grid");
                }
                curNode = nextNode;
                lastIndex = curIndex;
            }

            for(const base::Pose2D &p : cwp.poses)
            {
                //start is already corrected to be in the middle of a cell, thus cwp.pose.position should not be corrected
                base::Vector3d pos(p.position.x() + start.x(), p.position.y() + start.y(), start.z());
                pos.z() = curNode->getHeight();
                // HACK this overwrite avoids wrong headings in trajectory
                //See ticket: https://git.hb.dfki.de/entern/ugv_nav4d/issues/1
                if(setZToZero)
                    pos.z() = 0.0;
                //this just changes the z-coordinate (slightly wasteful to use Affine3d for that, but not inside critical loop)
                Eigen::Vector3d pos_Body = plan2Body.inverse(Eigen::Isometry) * pos;
                if(positions.empty() || !(positions.back().isApprox(pos_Body)))
                {
                    //need to offset by start because the poses are relative to (0/0)
                    positions.emplace_back(pos_Body);
                }
            }
        }
        if (mobilityConfig.remove_goal_offset == true &&
            i == indexOfMotionToUpdate)
        {
            LOG_INFO_S << "Original spline end position: " << positions[positions.size()-1];
            double goal_offset_x = (goalPos.x() - positions[positions.size()-1].x()) / (positions.size()-1);
            double goal_offset_y = (goalPos.y() - positions[positions.size()-1].y()) / (positions.size()-1);

            for (int j{0}; j < positions.size(); j++){
                positions[j].x() += j*goal_offset_x;
                positions[j].y() += j*goal_offset_y;
            }
            LOG_INFO_S << "Updated spline end position: " << positions[positions.size()-1];
            goal_position_updated = true;
        }

        curPart.spline.interpolate(positions);

#ifdef ENABLE_V3DD_DRAWINGS
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
    //                 pos = mlsGrid->getLocalFrame().inverse(Eigen::Isometry) * pos;
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
            if (curMotion.type == Motion::Type::MOV_BACKWARD)
            {
                curPart.speed = -mobilityConfig.translationSpeed;
            }
            else
            {
                curPart.speed = mobilityConfig.translationSpeed;
            }
            SubTrajectory curPartSub(curPart);
            switch (curMotion.type) {
                case Motion::Type::MOV_FORWARD:
                    curPartSub.driveMode = DriveMode::ModeAckermann;
                    break;
                case Motion::Type::MOV_BACKWARD:
                    curPartSub.driveMode = DriveMode::ModeAckermann;
                    break;
                case Motion::Type::MOV_LATERAL:
                    curPartSub.driveMode = DriveMode::ModeSideways;
                    break;
            }
            result.push_back(curPartSub);

            if (goal_position_updated){
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
                if (goalHeading < 0){
                    goalHeading += 2*M_PI;
                }
                goalPose.orientation  = goalHeading;

                if (std::abs(goalPose.orientation - startPose.orientation) > 0.01){
                    std::vector<base::Angle> angles;
                    angles.emplace_back(base::Angle::fromRad(startPose.orientation));
                    angles.emplace_back(base::Angle::fromRad(goalPose.orientation));

                    subtraj.interpolate(goalPose,angles);
                    subtraj.startPose     = startPose;
                    subtraj.goalPose      = goalPose;
                    result.push_back(subtraj);
                }
                goal_position_updated = false;
            }
            start = curPart.spline.getEndPoint();
        }
    }
}

const maps::grid::TraversabilityMap3d<traversability_generator3d::TravGenNode*>& EnvironmentXYZTheta::getTraversabilityMap() const
{
    return travGen.getTraversabilityMap();
}

const maps::grid::TraversabilityMap3d<traversability_generator3d::TravGenNode*>& EnvironmentXYZTheta::getObstacleMap() const
{
    return obsGen.getTraversabilityMap();
}

const EnvironmentXYZTheta::MLGrid& EnvironmentXYZTheta::getMlsMap() const
{
    return *mlsGrid;
}

const PreComputedMotions& EnvironmentXYZTheta::getAvailableMotions() const
{
    return availableMotions;
}

double EnvironmentXYZTheta::getAvgSlope(std::vector<const traversability_generator3d::TravGenNode*> path) const
{
    if(path.size() <= 0)
    {
        throw std::runtime_error("Requested slope of path with length zero.");
        return 0;
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

    Dijkstra::computeCost(startXYZNode->getUserData().travNode, costToStart, travConf);
    Dijkstra::computeCost(goalXYZNode->getUserData().travNode, costToEnd, travConf);
    assert(costToStart.size() == costToEnd.size());

    //FIXME this should be a config value?!
    const double maxDist = 99999999; //big enough to never occur in reality. Small enough to not cause overflows when used by accident.
    travNodeIdToDistance.clear();
    travNodeIdToDistance.resize(travGen.getNumNodes(), Distance(maxDist, maxDist));

    for(const auto pair : costToStart)
    {
        const traversability_generator3d::TravGenNode* node = static_cast<const traversability_generator3d::TravGenNode*>(pair.first);
        const double cost = pair.second;
        travNodeIdToDistance[node->getUserData().id].distToStart = cost;
    }

    for(const auto pair : costToEnd)
    {
        const traversability_generator3d::TravGenNode* node = static_cast<const traversability_generator3d::TravGenNode*>(pair.first);
        const double cost = pair.second;
        travNodeIdToDistance[node->getUserData().id].distToGoal = cost;
    }
}

traversability_generator3d::TraversabilityGenerator3d& EnvironmentXYZTheta::getTravGen()
{
    return travGen;
}

traversability_generator3d::TraversabilityGenerator3d& EnvironmentXYZTheta::getObstacleGen()
{
    return obsGen;
}

void EnvironmentXYZTheta::setTravConfig(const traversability_generator3d::TraversabilityConfig& cfg)
{
    travConf = cfg;
}


traversability_generator3d::TravGenNode* EnvironmentXYZTheta::findObstacleNode(const traversability_generator3d::TravGenNode* travNode) const
{
    Eigen::Vector3d posWorld;
    travGen.getTraversabilityMap().fromGrid(travNode->getIndex(), posWorld, travNode->getHeight(), false);
    maps::grid::Index idxObstMap;
    obsGen.getTraversabilityMap().toGrid(posWorld, idxObstMap, false);


    traversability_generator3d::TravGenNode *obstNode = nullptr;
    double minDist = std::numeric_limits< double >::max();
    for(traversability_generator3d::TravGenNode* n: obsGen.getTraversabilityMap().at(idxObstMap))
    {
        double curDist = fabs(n->getHeight() - travNode->getHeight());
        if(curDist > minDist)
        {
            //we passed the minimal distance point
            break;
        }
        minDist = curDist;
        obstNode = n;
    }

    return obstNode;

}

std::shared_ptr<SubTrajectory> EnvironmentXYZTheta::findTrajectoryOutOfObstacle(const Eigen::Vector3d& start,
                                                                                double theta,
                                                                                const Eigen::Affine3d& ground2Body,
                                                                                base::Vector3d& outNewStart,
                                                                                double& outNewStartTheta)
{
    traversability_generator3d::TravGenNode* startTravNode = travGen.generateStartNode(start);

    if(!startTravNode->isExpanded())
    {
        LOG_INFO_S<< "cannot find trajectory out of obstacle, map not expanded";
        //this node should be expanded
        throw std::runtime_error("cannot find trajectory out of obstacle, map not expanded");
    }

    Eigen::Vector3d startPosWorld;
    travGen.getTraversabilityMap().fromGrid(startTravNode->getIndex(), startPosWorld, startTravNode->getHeight(), false);

    DiscreteTheta thetaD(theta, numAngles);
    traversability_generator3d::TravGenNode* startNodeObstMap = findObstacleNode(startTravNode);
    const maps::grid::Index startIdxObstMap =  startNodeObstMap->getIndex();

    if(!startNodeObstMap)
    {
        LOG_INFO_S<< "Unable to find obstacle node corresponding to trav node";
        throw std::runtime_error("unable to find obstacle node corresponding to trav node");
    }

    int bestMotionIndex = -1;
    std::vector<const traversability_generator3d::TravGenNode*> bestNodesOnPath;
    std::vector<base::Pose2D> bestPosesOnObstPath;
    int bestMotionObstacleCount = std::numeric_limits<int>::max();

    bool intermediateStepsOk = true;
    const auto& motions = availableMotions.getMotionForStartTheta(thetaD);
    for(size_t i = 0; i < motions.size(); ++i)
    {
        const ugv_nav4d::Motion &motion(motions[i]);
        const traversability_generator3d::TravGenNode* currentObstNode = startNodeObstMap;
        std::vector<const traversability_generator3d::TravGenNode*> nodesOnPath;
        std::vector<base::Pose2D> posesOnObstPath;

        nodesOnPath.push_back(currentObstNode);

        base::Pose2D firstPose;

        //Currently the function only selects a single motion so the pointturn will not help us.
        //TODO: If multiple motions are agreegated to get the final recovery trajectory then pointturns can be used.
        //NOTE: Pointturns have no intermediateStepsObstMap, so are skipped at the moment.
        if (motion.type == ugv_nav4d::Motion::MOV_POINTTURN){
             continue;
        }
        firstPose = motion.intermediateStepsObstMap[0].pose;
        firstPose.position += startPosWorld.head<2>();
        posesOnObstPath.push_back(firstPose);

        intermediateStepsOk = true;
        for(size_t j = 1; j < motion.intermediateStepsObstMap.size(); ++j)
        {
            const PoseWithCell& pwc = motion.intermediateStepsObstMap[j];
            //diff is always a full offset to the start position
            const maps::grid::Index newIndex =  startIdxObstMap + pwc.cell;
            currentObstNode = currentObstNode->getConnectedNode(newIndex);
            if(currentObstNode == nullptr)
            {
                intermediateStepsOk = false;
                break;
            }
            nodesOnPath.push_back(currentObstNode);

            base::Pose2D curPose = pwc.pose;
            curPose.position += startPosWorld.head<2>();
            posesOnObstPath.push_back(curPose);

        }

        if(!intermediateStepsOk)
        {
            continue;
        }


        //check if the endpose is outside an obstacle
        std::vector<const traversability_generator3d::TravGenNode*> endPosePath;
        std::vector<base::Pose2D> endPosePoses;
        endPosePath.push_back(currentObstNode);
        Eigen::Vector3d endPosWorld;
        obsGen.getTraversabilityMap().fromGrid(currentObstNode->getIndex(), endPosWorld, currentObstNode->getHeight(), false);
        base::Pose2D endPose;
        endPose.position = endPosWorld.topRows(2);
        endPose.orientation = motions[i].endTheta.getRadian();
        endPosePoses.push_back(endPose);
        PathStatistic endPoseStats(travConf);
        endPoseStats.calculateStatistics(endPosePath, endPosePoses, obsGen.getTraversabilityMap());
        if(endPoseStats.getRobotStats().getNumObstacles() > 0 ||
           endPoseStats.getRobotStats().getNumFrontiers() > 0)
        {
            //this path ends in an obstacle
            continue;
        }


        PathStatistic stats(travConf);
        stats.calculateStatistics(nodesOnPath, posesOnObstPath, obsGen.getTraversabilityMap());
        const int obstacleCount = stats.getRobotStats().getNumObstacles() + stats.getRobotStats().getNumFrontiers();

        if(obstacleCount < bestMotionObstacleCount)
        {
            bestMotionObstacleCount = obstacleCount;
            bestMotionIndex = i;
            bestNodesOnPath = nodesOnPath;
            bestPosesOnObstPath = posesOnObstPath;

            outNewStart = endPosWorld;
            outNewStartTheta = motions[bestMotionIndex].endTheta.getRadian();

        }
    }

    base::Trajectory trajectory;

    if(bestMotionIndex != -1)
    {
        //turn the poses into a spline
        std::vector<base::Vector3d> positions;

        for(const base::Pose2D &p : bestPosesOnObstPath)
        {
            base::Vector3d position(p.position.x(), p.position.y(), startPosWorld.z());

            // HACK this overwrite avoids wrong headings in trajectory
            // TODO ideally, this should interpolate the actual height (but at the moment this would only make a difference in visualization)
            position.z() = 0.0;
            Eigen::Vector3d pos_Body = ground2Body.inverse(Eigen::Isometry) * position;

            positions.push_back(pos_Body);
        }
        trajectory.spline.interpolate(positions);
        trajectory.speed = motions[bestMotionIndex].type == Motion::Type::MOV_BACKWARD? -mobilityConfig.translationSpeed : mobilityConfig.translationSpeed;
#ifdef ENABLE_V3DD_DRAWINGS
            V3DD::COMPLEX_DRAWING([&]()
            {
                for(base::Vector3d pos : positions)
                {
    //                 pos = mlsGrid->getLocalFrame().inverse(Eigen::Isometry) * pos;
                    V3DD::DRAW_CYLINDER("ugv_nav4d_outOfObstacleTrajectory", pos,  base::Vector3d(0.02, 0.02, 0.2), V3DD::Color::blue);
                }
            });
#endif
    }
    else
    {
        LOG_INFO_S<< "NO WAY OUT, ROBOT IS STUCK!";
        LOG_INFO_S<< "NO WAY OUT, ROBOT IS STUCK!";
        LOG_INFO_S<< "NO WAY OUT, ROBOT IS STUCK!";
        return nullptr;
    }

    std::shared_ptr<SubTrajectory> subTraj(new SubTrajectory(trajectory));
    return subTraj;
}



}
