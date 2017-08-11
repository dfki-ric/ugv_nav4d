#include "EnvironmentXYZTheta.hpp"
#include <sbpl/planners/planner.h>
#include <sbpl/utils/mdpconfig.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <base/Pose.hpp>
#include <fstream>
#include <dwa/SubTrajectory.hpp>
#include <backward/backward.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.h>
#include <vizkit3d_debug_drawings/DebugDrawingColors.h>
#include "CollisionCheck.hpp"
#include "PathStatistics.hpp"

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

#define PRINT_VAR(var) \
    std::cout << #var << ": " << var << std::endl;
    


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
    travGen.setMLSGrid(mlsGrid);
    searchGrid.setResolution(Eigen::Vector2d(travConf.gridResolution, travConf.gridResolution));
    searchGrid.extend(travGen.getTraversabilityMap().getNumCells());
    robotHalfSize << travConf.robotSizeX / 2, travConf.robotSizeY / 2, travConf.robotHeight/2;
    
    UGV_DEBUG(
        debugData.setTravConf(travConf);
        debugData.setTravGen(&travGen);
        debugData.setMlsGrid(mlsGrid);
    )
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
        delete p;
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
}

void EnvironmentXYZTheta::updateMap(boost::shared_ptr< EnvironmentXYZTheta::MLGrid > mlsGrid)
{
    if(this->mlsGrid && this->mlsGrid->getResolution() != mlsGrid->getResolution())
        throw std::runtime_error("EnvironmentXYZTheta::updateMap : Error got MLSMap with different resolution");
    
    travGen.setMLSGrid(mlsGrid);
    this->mlsGrid = mlsGrid;
    
    UGV_DEBUG(
        debugData.setMlsGrid(mlsGrid);
    )

    clear();
}

EnvironmentXYZTheta::XYZNode* EnvironmentXYZTheta::createNewXYZState(TravGenNode* travNode)
{
    XYZNode *xyzNode = new XYZNode(travNode->getHeight(), travNode->getIndex());
    xyzNode->getUserData().travNode = travNode;
    searchGrid.at(travNode->getIndex()).insert(xyzNode);

    return xyzNode;
}


EnvironmentXYZTheta::ThetaNode* EnvironmentXYZTheta::createNewStateFromPose(const Eigen::Vector3d& pos, double theta, XYZNode **xyzBackNode)
{
    TravGenNode *travNode = travGen.generateStartNode(pos);
    if(!travNode)
    {
        cout << "createNewStateFromPose: Error Pose " << pos.transpose() << " is out of grid" << endl;
        throw runtime_error("Pose is out of grid");
    }

    //check if intitial patch is unknown
    if(travNode->isExpanded() && (travNode->getType() != maps::grid::TraversabilityNodeBase::TRAVERSABLE))
    {
        cout << "createNewStateFromPose: Error Pose " << pos.transpose() << " is not traversable" << endl;
        throw runtime_error("Pose is not traversable");
    }
    else
    {
        if(!travGen.expandNode(travNode))
        {
            cout << "createNewStateFromPose: Error Pose " << pos.transpose() << " is not traversable" << endl;
            throw runtime_error("Pose is not traversable");
        }
        travNode->setNotExpanded();
    }
    
    XYZNode *xyzNode = createNewXYZState(travNode);
    
    DiscreteTheta thetaD(theta, numAngles);
    
    if(xyzBackNode)
        *xyzBackNode = xyzNode;
    
    return createNewState(thetaD, xyzNode);
}


void EnvironmentXYZTheta::setGoal(const Eigen::Vector3d& goalPos, double theta)
{
    
    CLEAR_DRAWING("env_goalPos");
    DRAW_ARROW("env_goalPos", goalPos, base::Quaterniond(Eigen::AngleAxisd(M_PI, base::Vector3d::UnitX())),
               base::Vector3d(1,1,1), vizkit3dDebugDrawings::Color::red);
    
    if(!startXYZNode)
        throw std::runtime_error("Error, start needs to be set before goal");
    
    goalThetaNode = createNewStateFromPose(goalPos, theta, &goalXYZNode);
    
    if(!checkOrientationAllowed(goalXYZNode->getUserData().travNode, theta))
        throw std::runtime_error("Goal orientation not allowed due to slope");
    
    if(!CollisionCheck::checkCollision(goalXYZNode->getUserData().travNode, theta,
                                       mlsGrid, robotHalfSize, travGen))
    {
        if(travConf.ignoreCollisions)
        {
            std::cout << "WARNING: Goal inside obstacle\n";
        }
        else
        {
            throw std::runtime_error("Goal inside obstacle");
        }
    }
    
    //NOTE If we want to precompute the heuristic (precomputeCost()) we need to expand 
    //     the whole travmap beforehand.
    
    std::cout << "GOAL IS: " << goalPos.transpose() << std::endl;
    
    travGen.expandAll(startXYZNode->getUserData().travNode);
    std::cout << "All expanded " << std::endl;
    precomputeCost();
    std::cout << "Heuristic computed" << std::endl;
}

void EnvironmentXYZTheta::setStart(const Eigen::Vector3d& startPos, double theta)
{
    CLEAR_DRAWING("env_startPos");
    DRAW_ARROW("env_startPos", startPos, base::Quaterniond(Eigen::AngleAxisd(M_PI, base::Vector3d::UnitX())),
               base::Vector3d(1,1,1), vizkit3dDebugDrawings::Color::blue);
    
    startThetaNode = createNewStateFromPose(startPos, theta, &startXYZNode);
   
    //NOTE those checks are useless, the robot odometry/slam/trajectoryFollower are not perfect
    //     thus we can end up in locations where we shouldn't be.
//     if(!checkOrientationAllowed(startXYZNode->getUserData().travNode, theta))
//         throw std::runtime_error("Start orientation not allowed due to slope");
    
//     if(!CollisionCheck::checkCollision(startXYZNode->getUserData().travNode, theta, mlsGrid, robotHalfSize, travGen))
//     {
//         std::cout << "START INSIDE OBSTACLE" << std::endl;
//     }
    
    std::cout << "START IS: " << startPos.transpose() << std::endl;
    
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
    const Hash &sourceHash(idToHash[stateID]);
    const XYZNode *sourceNode = sourceHash.node;
    const TravGenNode* travNode = sourceNode->getUserData().travNode;
    const ThetaNode *sourceThetaNode = sourceHash.thetaNode;
    
    const double sourceToGoalDist = travNodeIdToDistance[travNode->getUserData().id].distToGoal;
    const double timeTranslation = sourceToGoalDist / mobilityConfig.mSpeed;
    const double timeRotation = sourceThetaNode->theta.shortestDist(goalThetaNode->theta).getRadian() / mobilityConfig.mTurningSpeed;
    
    const int result = floor(std::max(timeTranslation, timeRotation) * costScaleFactor);
    if(result < 0)
    {
        
        PRINT_VAR(sourceToGoalDist);
        PRINT_VAR( mobilityConfig.mSpeed);
        PRINT_VAR(timeTranslation);
        PRINT_VAR(sourceThetaNode->theta.shortestDist(goalThetaNode->theta).getRadian());
        PRINT_VAR(mobilityConfig.mTurningSpeed);
        PRINT_VAR(timeRotation);
        PRINT_VAR(result);
        PRINT_VAR(travNode->getUserData().id);
        PRINT_VAR(travNode->getType());
        
    }
    oassert(result >= 0);
    return result;
}

int EnvironmentXYZTheta::GetStartHeuristic(int stateID)
{
    const Hash &targetHash(idToHash[stateID]);
    const XYZNode *targetNode = targetHash.node;
    const TravGenNode* travNode = targetNode->getUserData().travNode;
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

TravGenNode *EnvironmentXYZTheta::movementPossible(TravGenNode *fromTravNode, const maps::grid::Index &fromIdx, const maps::grid::Index &toIdx)
{
    if(toIdx == fromIdx)
        return fromTravNode;
    
    //get trav node associated with the next index
    TravGenNode *targetNode = fromTravNode->getConnectedNode(toIdx);
    if(!targetNode)
    {
        return nullptr;
        //FIXME this should never happen but it does on the garage map with 0.5 resolution
        throw std::runtime_error("should not happen");
    }
    
    if(!checkExpandTreadSafe(targetNode))
    {
        return nullptr;
    }
    
    //NOTE this check cannot be done before checkExpandTreadSafe because the type will be determined
    //     during the expansion. Beforehand the type is undefined
    if(targetNode->getType() != maps::grid::TraversabilityNodeBase::TRAVERSABLE)
    {
        return nullptr;
    }  
    
        
    //TODO add additionalCosts if something is near this node etc
    return targetNode;
}

bool EnvironmentXYZTheta::checkExpandTreadSafe(TravGenNode * node)
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
            //FIXME if expandNode throws an exeception we may never unlock
            //directly returning from insde omp critical is forbidden
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


void EnvironmentXYZTheta::GetSuccs(int SourceStateID, vector< int >* SuccIDV, vector< int >* CostV, vector< size_t >& motionIdV)
{
    SuccIDV->clear();
    CostV->clear();
    motionIdV.clear();
    const Hash &sourceHash(idToHash[SourceStateID]);
    const XYZNode *const sourceNode = sourceHash.node;
    
    COMPLEX_DRAWING(
        const TravGenNode* node = sourceNode->getUserData().travNode;
        Eigen::Vector3d pos((node->getIndex().x() + 0.5) * travConf.gridResolution,
                             (node->getIndex().y() + 0.5) * travConf.gridResolution,
                              node->getHeight());
        pos = mlsGrid->getLocalFrame().inverse(Eigen::Isometry) * pos;
        DRAW_WIREFRAME_BOX("successors", pos, base::Vector3d(mlsGrid->getResolution().x() / 2.0, mlsGrid->getResolution().y() / 2.0,
                           0.05), vizkit3dDebugDrawings::Color::blue);
    );

    
    
    const ThetaNode *const thetaNode = sourceHash.thetaNode;
    const maps::grid::Index sourceIndex = sourceNode->getIndex();

    
    TravGenNode *curTravNode = sourceNode->getUserData().travNode;
    if(!curTravNode->isExpanded())
    {
        //current node is not drivable
        if(!travGen.expandNode(curTravNode))
        {
            return;
        }
    }

    const auto& motions = availableMotions.getMotionForStartTheta(thetaNode->theta);
    #pragma omp parallel for schedule(dynamic, 5) if(travConf.parallelismEnabled)
    for(size_t i = 0; i < motions.size(); ++i)
    {
        const Motion &motion = motions[i];
        TravGenNode *travNode = sourceNode->getUserData().travNode;
        maps::grid::Index curIndex = sourceNode->getIndex();
        std::vector<TravGenNode*> nodesOnPath;
        bool intermediateStepsOk = true;

        for(const PoseWithCell &diff : motion.intermediateSteps)
        {
            //diff is always a full offset to the start position
            const maps::grid::Index newIndex =  sourceIndex + diff.cell;
            travNode = movementPossible(travNode, curIndex, newIndex);
            nodesOnPath.push_back(travNode);
            
            if(!travNode ||
               !checkOrientationAllowed(travNode, diff.pose.orientation))
            {
                intermediateStepsOk = false;
                break;
            }
            curIndex = newIndex;
        }
        
        if(!intermediateStepsOk)
            continue;

        if(!travConf.ignoreCollisions && !checkCollisions(nodesOnPath, motion))
            continue;
        
        //goal from source to the end of the motion was valid
        XYZNode *successXYNode = nullptr;
        ThetaNode *successthetaNode = nullptr;
        
        //WARNING This becomes a critical section if several motion primitives
        //        share the same finalPos.
        //        As long as this is not the case this section should be save.
        const maps::grid::Index finalPos(sourceIndex.x() + motion.xDiff, sourceIndex.y() + motion.yDiff);
        
        #pragma omp critical(searchGridAccess) 
        {
            const auto &candidateMap = searchGrid.at(finalPos);

            if(travNode->getIndex() != finalPos)
                throw std::runtime_error("Internal error, indexes do not match");
            
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
                successXYNode = createNewXYZState(travNode); //modifies searchGrid at travNode->getIndex()
            }
        }

        #pragma omp critical(thetaToNodesAccess) //TODO reduce size of critical section
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
            case SlopeMetric::AVG_SLOPE:
            {
                const double slopeFactor = getAvgSlope(nodesOnPath) * travConf.slopeMetricScale;
                cost = motion.baseCost + motion.baseCost * slopeFactor;
                break;
            }
            case SlopeMetric::MAX_SLOPE:
            {
                const double slopeFactor = getMaxSlope(nodesOnPath) * travConf.slopeMetricScale;
                cost = motion.baseCost + motion.baseCost * slopeFactor;
                break;
            }
            case SlopeMetric::TRIANGLE_SLOPE:
            {
                //assume that the motion is a straight line, extrapolate into third dimension
                //by projecting onto a plane that connects start and end cell.
                const double heightDiff = std::abs(sourceNode->getHeight() - successXYNode->getHeight());
                //not perfect but probably more exact than the slope factors above
                const double approxMotionLen3D = std::sqrt(std::pow(motion.translationlDist, 2) + std::pow(heightDiff, 2));
                assert(approxMotionLen3D >= motion.translationlDist);//due to triangle inequality
                const double translationalVelocity = std::min(mobilityConfig.mSpeed, motion.speed);
                cost = Motion::calculateCost(approxMotionLen3D, motion.angularDist, translationalVelocity,
                                             mobilityConfig.mTurningSpeed, motion.costMultiplier);
                break;
            }
            case SlopeMetric::NONE:
                cost = motion.baseCost;
                break;
            default:
                throw std::runtime_error("unknown slope metric selected");
        }

        PathStatistic statistic(travConf);
        statistic.calculateStatistics(nodesOnPath);
        
        if(statistic.getNumObstacles())
        {
            const double outer_radius = travConf.costFunctionDist;
            double minDistToRobot = statistic.getMinDistToObstacles();
            minDistToRobot = std::min(outer_radius, minDistToRobot);
            double impactFactor = (outer_radius - minDistToRobot) / outer_radius;
            oassert(impactFactor < 1.001 && impactFactor >= 0);
            
            cost += cost * impactFactor;
        }

        if(statistic.getNumFrontiers())
        {
            const double outer_radius = travConf.costFunctionDist;
            double minDistToRobot = statistic.getMinDistToFrontiers();
            minDistToRobot = std::min(outer_radius, minDistToRobot);
            double impactFactor = (outer_radius - minDistToRobot) / outer_radius;
            oassert(impactFactor < 1.001 && impactFactor >= 0);

            cost += cost * impactFactor;
        }
        
        oassert(int(cost) >= motion.baseCost);
        oassert(motion.baseCost > 0);
        
        const int iCost = (int)cost;
        #pragma omp critical(updateData)
        {
            SuccIDV->push_back(successthetaNode->id);
            CostV->push_back(iCost);
            motionIdV.push_back(motion.id);
        }
    } 
}

bool EnvironmentXYZTheta::checkOrientationAllowed(const TravGenNode* node,
                                const base::Orientation2D& orientationRad) const
{   
    //otherwise something whent wrong when generating the map
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


bool EnvironmentXYZTheta::checkCollisions(const std::vector<TravGenNode*>& path,
                                          const Motion& motion) const
{
    oassert(motion.intermediateSteps.size() == path.size());
    
    for(size_t i = 0; i < path.size(); ++i)
    {
        const TravGenNode* node(path[i]);
        
        //path contains the final element while intermediatePoses does not.
        const double zRot = motion.intermediateSteps[i].pose.orientation;
        if(!CollisionCheck::checkCollision(node, zRot, mlsGrid, robotHalfSize, travGen))
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


void EnvironmentXYZTheta::getTrajectory(const vector< int >& stateIDPath, vector< base::Trajectory >& result, const Eigen::Affine3d &plan2Body)
{
    if(stateIDPath.size() < 2)
        return;
    
    result.clear();

    base::Trajectory curPart;
    
    CLEAR_DRAWING("trajectory");
    
    UGV_DEBUG(
        std::vector<TravGenNode*> debugNodes;
    )
    
    for(size_t i = 0; i < stateIDPath.size() - 1; ++i)
    {
        const Motion& curMotion = getMotion(stateIDPath[i], stateIDPath[i+1]);
        const maps::grid::Vector3d start = getStatePosition(stateIDPath[i]);
        const Hash &startHash(idToHash[stateIDPath[i]]);
        const maps::grid::Index startIndex(startHash.node->getIndex());
        maps::grid::Index lastIndex = startIndex;
        TravGenNode *curNode = startHash.node->getUserData().travNode;     
        
        curPart.attributes.names.push_back("start_" + std::to_string(i));
        curPart.attributes.elements.push_back(std::to_string(start.x()) + "_" + std::to_string(start.y()) + "_" + std::to_string(start.z()));
        
        size_t pwcIdx = 0;
        std::vector<base::Vector3d> positions;
        for(const CellWithPoses &cwp : curMotion.fullSplineSamples)
        {
            maps::grid::Index curIndex = startIndex + cwp.cell;

            if(curIndex != lastIndex)
            {
                TravGenNode *nextNode = curNode->getConnectedNode(curIndex);
                if(!nextNode)
                {
                    for(auto *n : curNode->getConnections())
                        std::cout << "Con Node " << n->getIndex().transpose() << std::endl;;
                    throw std::runtime_error("Internal error, trajectory is not continous on tr grid");
                }

                curNode = nextNode;

                lastIndex = curIndex;
            }
            
            for(const base::Pose2D &p : cwp.poses)
            {
                //start is already corrected to be in the middle of a cell, thus pwc.pose.position should not be corrected
                base::Vector3d pos(p.position.x() + start.x(), p.position.y() + start.y(), start.z());

                pos.z() = curNode->getHeight();
                Eigen::Vector3d pos_Body = plan2Body.inverse() * pos;

                if(positions.empty() || !(positions.back().isApprox(pos_Body)))
                {
                    //need to offset by start because the poses are relative to (0/0)
                    positions.emplace_back(pos_Body);
                    curPart.attributes.names.push_back("motion_" + std::to_string(i) + "_" + std::to_string(pwcIdx++));
                    curPart.attributes.elements.push_back(std::to_string(pos_Body.x()) + "_" + std::to_string(pos_Body.y()) + "_" + std::to_string(pos_Body.z()));
                }
            }
        }
        
        curPart.spline.interpolate(positions);
        
        COMPLEX_DRAWING(
            for(base::Vector3d pos : positions)
            {
//                 pos = mlsGrid->getLocalFrame().inverse(Eigen::Isometry) * pos;
                DRAW_CYLINDER("trajectory", pos,  base::Vector3d(0.02, 0.02, 0.2), vizkit3dDebugDrawings::Color::cyan);
            }
        );
        
        
        curPart.speed = curMotion.type == Motion::Type::MOV_BACKWARD? -curMotion.speed : curMotion.speed;
        result.push_back(curPart);
    }
    
}

maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase* > EnvironmentXYZTheta::getTraversabilityBaseMap() const
{
    return travGen.getTraversabilityBaseMap();
}

const maps::grid::TraversabilityMap3d<TravGenNode*>& EnvironmentXYZTheta::getTraversabilityMap() const
{
    return travGen.getTraversabilityMap();
}

const EnvironmentXYZTheta::MLGrid& EnvironmentXYZTheta::getMlsMap() const
{
    return *mlsGrid;
}

const PreComputedMotions& EnvironmentXYZTheta::getAvailableMotions() const
{
    return availableMotions;
}

double EnvironmentXYZTheta::getAvgSlope(std::vector<TravGenNode*> path) const
{
    double slopeSum = 0;
    for(TravGenNode* node : path)
    {
        slopeSum += node->getUserData().slope; 
    }
    const double avgSlope = slopeSum / path.size();
    return avgSlope;
}

double EnvironmentXYZTheta::getMaxSlope(std::vector<TravGenNode*> path) const
{
    const TravGenNode* maxElem =  *std::max_element(path.begin(), path.end(),
                                  [] (TravGenNode* lhs, TravGenNode* rhs) 
                                  {
                                    return lhs->getUserData().slope < rhs->getUserData().slope;
                                  });
    return maxElem->getUserData().slope;
}

void EnvironmentXYZTheta::precomputeCost()
{
    std::vector<double> costToStart;
    std::vector<double> costToEnd;
    
    //FIXME test if using double max causes problems, if not, use it.
    const double maxDist = 99999999; //big enough to never occur in reality. Small enough to not cause overflows when used by accident.
    
    travGen.dijkstraComputeCost(startXYZNode->getUserData().travNode, costToStart, maxDist);
    travGen.dijkstraComputeCost(goalXYZNode->getUserData().travNode, costToEnd, maxDist);
    
    assert(costToStart.size() == costToEnd.size());
    
    travNodeIdToDistance.clear();
    travNodeIdToDistance.resize(costToStart.size(), Distance(maxDist, maxDist));
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


TraversabilityGenerator3d& EnvironmentXYZTheta::getTravGen()
{
    return travGen;
}

void EnvironmentXYZTheta::setTravConfig(const TraversabilityConfig& cfg)
{
    travConf = cfg;
}


}
