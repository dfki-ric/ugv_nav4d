#include "Planner.hpp"
#include <sbpl/planners/araplanner.h>
#include <sbpl/utils/mdpconfig.h>
#include <maps/grid/MultiLevelGridMap.hpp>
#include <base/Eigen.hpp>
#include "PlannerDump.hpp"
#include <omp.h>
#include <cmath>
#include <base-logging/Logging.hpp>
#include "Logger.hpp"
#include <deque>
#include <unordered_set>

#ifdef ENABLE_DEBUG_DRAWINGS
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>
#include <vizkit3d_debug_drawings/DebugDrawingColors.hpp>
#endif

using namespace maps::grid;
using trajectory_follower::SubTrajectory;

namespace ugv_nav4d
{


Planner::Planner(const sbpl_spline_primitives::SplinePrimitivesConfig& primitiveConfig, const traversability_generator3d::TraversabilityConfig& traversabilityConfig,
        const Mobility& mobility, const PlannerConfig& plannerConfig) :
    splinePrimitiveConfig(primitiveConfig),
    mobility(mobility),
    plannerConfig(plannerConfig)
{
    setTravConfig(traversabilityConfig);
}

void Planner::enablePathStatistics(bool enable){
    if (env){
        env->enablePathStatistics(enable);
    }
}

bool Planner::calculateGoal(Eigen::Vector3d& goal_translation, const double yaw) noexcept
{
    if (tryGoal(goal_translation, yaw)){
        return true;
    }

    traversability_generator3d::TravGenNode* travNode = env->findMatchingTraversabilityPatchAt(goal_translation);
    if (!travNode){
        return false;
    }

    auto trMap = env->getTraversabilityMap();
    auto pos1 = travNode->getPosition(*trMap);

    std::deque<maps::grid::TraversabilityNodeBase*> candidates;
    std::unordered_set<maps::grid::TraversabilityNodeBase *> visited;

    candidates.push_back(travNode);
    visited.insert(travNode);

    while(!candidates.empty())
    {
        auto *node = candidates.front();
        candidates.pop_front();

        for(auto *n : node->getConnections()){
            if (visited.count(n)) continue;  // Skip visited

            auto pos2 = n->getPosition(*trMap);
            if ((pos1 - pos2).norm() > mobility.searchRadius){
                continue;
            }

            if (tryGoal(pos2, yaw)){
                goal_translation = pos2;
                LOG_INFO_S << "Estimated Goal Position: " << goal_translation.transpose();
                return true;
            }

            candidates.push_back(n);
            visited.insert(n);
        }
    }

    return false; // Add this to cover all control paths
}


void Planner::updateObstacleHulls(const std::vector<std::vector<Eigen::Vector2d>> hulls_in_map){
    env->updateObstacleHulls(hulls_in_map);
}


bool Planner::tryGoal(const Eigen::Vector3d& translation, const double yaw) noexcept
{
    try
    {
        env->setGoal(translation, yaw);
    }
    catch(const std::runtime_error& ex)
    {
        LOG_ERROR_S << "Caught exception while setting goal pose:"  << ex.what();
        return false;
    }
    return true;
}

Planner::PLANNING_RESULT Planner::plan(const base::Time& maxTime, const base::samples::RigidBodyState& start_pose,
                                       const base::samples::RigidBodyState& end_pose,
                                       std::vector<SubTrajectory>& resultTrajectory2D,
                                       std::vector<SubTrajectory>& resultTrajectory3D,
                                       bool dumpOnError, bool dumpOnSuccess)
{

    LOG_DEBUG_S << "Planning with " << plannerConfig.numThreads << " threads";
    omp_set_num_threads(plannerConfig.numThreads);
#ifdef ENABLE_DEBUG_DRAWINGS
    V3DD::CLEAR_DRAWING("ugv_nav4d_successors");
#endif
    if(!env)
    {
        LOG_ERROR_S << "Planner::plan : Error : No map was set";
        return NO_MAP;
    }

    resultTrajectory2D.clear();
    resultTrajectory3D.clear();
    env->clear();

    if(!planner)
        planner.reset(new ARAPlanner(env.get(), true));


    Eigen::Affine3d ground2Body(Eigen::Affine3d::Identity());
    ground2Body.translation() = Eigen::Vector3d(0, 0, -traversabilityConfig.distToGround);

    base::samples::RigidBodyState startbody2Mls = start_pose;
    base::samples::RigidBodyState endbody2Mls = end_pose;

    startbody2Mls.setTransform(startbody2Mls.getTransform());
    endbody2Mls.setTransform(endbody2Mls.getTransform());

    LOG_DEBUG_S << "start_pose position (raw): " << start_pose.position.transpose();
    LOG_DEBUG_S << "end_pose position (raw): " << end_pose.position.transpose();

    const Eigen::Affine3d startGround2Mls(startbody2Mls.getTransform() * ground2Body);
    const Eigen::Affine3d endGround2Mls(endbody2Mls.getTransform() *ground2Body);

    startbody2Mls.setTransform(startGround2Mls);
    endbody2Mls.setTransform(endGround2Mls);

    try
    {
        env->setStart(startGround2Mls.translation(), base::getYaw(Eigen::Quaterniond(startGround2Mls.linear())));
    }
    catch(const ugv_nav4d::ObstacleCheckFailed& ex)
    {
        LOG_ERROR_S << "Caught exception while setting start pose:"  << ex.what();
        if(dumpOnError)
            PlannerDump dump(*this, "start_inside_obstacle", maxTime, startbody2Mls, endbody2Mls);
        return START_INVALID;
    }
    catch(const std::runtime_error& ex)
    {
        LOG_ERROR_S << "Caught exception while setting start pose:"  << ex.what();
        if(dumpOnError)
            PlannerDump dump(*this, "bad_start", maxTime, startbody2Mls, endbody2Mls);
        return START_INVALID;
    }

    Eigen::Vector3d start_translation = startGround2Mls.translation();
    Eigen::Vector3d goal_translation = endGround2Mls.translation();

    if(!calculateGoal(goal_translation, base::getYaw(Eigen::Quaterniond(endGround2Mls.linear())))) {
        if(dumpOnError) {
            PlannerDump dump(*this, "bad_goal", maxTime, startbody2Mls, endbody2Mls);
        }
        return GOAL_INVALID;
    }

    //this has to happen after env->setStart and env->setGoal because those methods initialize the
    //StateID2IndexMapping which is accessed inside force_planning_from_scratch_and_free_memory().
    try
    {
        planner->force_planning_from_scratch_and_free_memory();
        planner->set_search_mode(plannerConfig.searchUntilFirstSolution);
    }
    catch(const SBPL_Exception& ex)
    {
        LOG_ERROR_S << "Caught SBPL exception: " << ex.what();
        return NO_SOLUTION;
    }


    MDPConfig mdp_cfg;

    if (!env->InitializeMDPCfg(&mdp_cfg)) {
        LOG_ERROR_S << "InitializeMDPCfg failed, start and goal id cannot be requested yet";
        return INTERNAL_ERROR;
    }
    if (planner->set_start(mdp_cfg.startstateid) == 0) {
        LOG_ERROR_S << "Failed to set start state";
        return INTERNAL_ERROR;
    }
    if (planner->set_goal(mdp_cfg.goalstateid) == 0) {
        LOG_ERROR_S << "Failed to set goal state";
        return INTERNAL_ERROR;
    }

    try
    {
        LOG_DEBUG_S << "Initial Epsilon: " << plannerConfig.initialEpsilon << ", steps: " << plannerConfig.epsilonSteps;
        planner->set_eps_step(plannerConfig.epsilonSteps);
        planner->set_initialsolution_eps(plannerConfig.initialEpsilon);

        solutionIds.clear();
        if(!planner->replan(maxTime.toSeconds(), &solutionIds))
        {
            LOG_DEBUG_S << "Number of state space expands: " << planner->get_n_expands();
            if(dumpOnError)
                PlannerDump dump(*this, "no_solution", maxTime, startbody2Mls, endbody2Mls);
            return NO_SOLUTION;
        }

        LOG_DEBUG_S << "num expands: " << planner->get_n_expands();
        LOG_DEBUG_S << "Epsilon is " << planner->get_final_epsilon();

        std::vector<PlannerStats> stats;

        planner->get_search_stats(&stats);
        env->getTrajectory(solutionIds, resultTrajectory2D, true, start_translation, goal_translation, end_pose.getYaw(), ground2Body);
        env->getTrajectory(solutionIds, resultTrajectory3D, false, start_translation, goal_translation,end_pose.getYaw(), ground2Body);
    }
    catch(const SBPL_Exception& ex)
    {
        LOG_ERROR_S << "Caught sbpl exception: " << ex.what();
        if(dumpOnError)
            PlannerDump dump(*this, "no_solution", maxTime, startbody2Mls, endbody2Mls);
        return NO_SOLUTION;
    }

    if(dumpOnSuccess)
        PlannerDump dump(*this, "success", maxTime, startbody2Mls, endbody2Mls);

    return FOUND_SOLUTION;
}

std::vector< Motion > Planner::getMotions() const
{
    return env->getMotions(solutionIds);
}

const std::shared_ptr<const traversability_generator3d::TravMap3d > Planner::getTraversabilityMap() const
{
    return env->getTraversabilityMap();
}

std::shared_ptr<SubTrajectory> Planner::findTrajectoryOutOfObstacle(const Eigen::Vector3d& start,
                                                                                double theta,
                                                                                const Eigen::Affine3d& ground2Body,
                                                                                bool setZToZero){
    if(env){
        try{
            return env->findTrajectoryOutOfObstacle(start, theta, ground2Body, setZToZero);
        }
        catch (const std::exception& e){
            LOG_ERROR_S << "Caught exception when finding trajectory out of obstacle: " << e.what();
            return nullptr;
        }
    }
    else {
        return nullptr;
    }
}

void Planner::setTravConfig(const traversability_generator3d::TraversabilityConfig& config)
{
    if(config.gridResolution != splinePrimitiveConfig.gridSize){
        LOG_ERROR_S << "Planner::Planner : Configuration error, grid resolution of Primitives and TraversabilityGenerator3d differ";
        throw std::runtime_error("Planner::Planner : Configuration error, grid resolution of Primitives and TraversabilityGenerator3d differ");
    }
    traversabilityConfig = config;
    if(env){
        env->setTravConfig(config);
    }
}

 void Planner::setPlannerConfig(const PlannerConfig& config)
 {
     plannerConfig = config;
 }

}
