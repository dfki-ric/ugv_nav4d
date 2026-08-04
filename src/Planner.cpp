#include "Planner.hpp"
#include <sbpl/planners/araplanner.h>
#include <sbpl/utils/mdpconfig.h>
#include <maps/grid/MultiLevelGridMap.hpp>
#include <base/Eigen.hpp>
#include "PlannerDump.hpp"
#include <omp.h>
#include <cmath>
#include <chrono>
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

bool Planner::calculateGoal(Eigen::Vector3d& goal_translation, const double yaw)
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

bool Planner::tryGoal(const Eigen::Vector3d& translation, const double yaw)
{
    try
    {
        env->setGoal(translation, yaw);
    }
    catch(const std::exception& ex)
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
    auto t_start_total = std::chrono::steady_clock::now();

    LOG_DEBUG_S << "Planning with " << plannerConfig.numThreads << " threads";
    omp_set_num_threads(plannerConfig.numThreads);
#if 0 //V3DD disabled: only ugv_nav4d_rs_input_states active
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

    // The Reeds-Shepp goal shot needs the RS final-path reconstruction to recreate the
    // (motion-less) goal edge, so it is only active when useReedsSheppFinalPath is set.
    if(plannerConfig.useReedsSheppGoalShot && !plannerConfig.useReedsSheppFinalPath)
        LOG_WARN_S << "useReedsSheppGoalShot requires useReedsSheppFinalPath; goal shot disabled.";
    env->setReedsSheppGoalShot(plannerConfig.useReedsSheppGoalShot && plannerConfig.useReedsSheppFinalPath,
                               plannerConfig.reedsSheppGoalShotMaxDistance,
                               plannerConfig.reedsSheppStepSize);

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

    auto t_env_init = std::chrono::steady_clock::now();

    try
    {
        env->setStart(startGround2Mls.translation(), base::getYaw(Eigen::Quaterniond(startGround2Mls.linear())));
    }
    catch(const ugv_nav4d::ObstacleCheckFailed& ex)
    {
        // ex.what() carries the accurate reason (real obstacle vs. disallowed orientation on a
        // partially traversable cell).
        LOG_ERROR_S << "Failed to set start pose: " << ex.what();
        if(dumpOnError)
            PlannerDump dump(*this, "bad_start", maxTime, startbody2Mls, endbody2Mls);
        return START_INVALID;
    }
    catch(const std::runtime_error& ex)
    {
        LOG_ERROR_S << "Caught exception while setting start pose:"  << ex.what();
        if(dumpOnError)
            PlannerDump dump(*this, "bad_start", maxTime, startbody2Mls, endbody2Mls);
        return START_INVALID;
    }

    auto t_set_start = std::chrono::steady_clock::now();

    Eigen::Vector3d start_translation = startGround2Mls.translation();
    Eigen::Vector3d goal_translation = endGround2Mls.translation();

    if(!calculateGoal(goal_translation, base::getYaw(Eigen::Quaterniond(endGround2Mls.linear())))) {
        if(dumpOnError) {
            PlannerDump dump(*this, "bad_goal", maxTime, startbody2Mls, endbody2Mls);
        }
        return GOAL_INVALID;
    }

    auto t_set_goal = std::chrono::steady_clock::now();

    // Always recreate the ARAPlanner to avoid stale state IDs.
    // env->clear() destroys all states, but force_planning_from_scratch_and_free_memory()
    // remembers old start/goal IDs from a previous run. If the new plan creates fewer
    // states, those old IDs exceed StateID2IndexMapping.size() causing "stateID is invalid".
    try
    {
        planner.reset(new ARAPlanner(env.get(), true));
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

    auto t_planner_setup = std::chrono::steady_clock::now();

    PLANNING_RESULT planning_res = NO_SOLUTION;
    int num_expands = 0;
    double final_epsilon = -1.0;
    auto t_replan_start = std::chrono::steady_clock::now();
    auto t_replan_end = t_replan_start;
    auto t_trajectory_extraction = t_replan_start;

    try
    {
        LOG_DEBUG_S << "Initial Epsilon: " << plannerConfig.initialEpsilon << ", steps: " << plannerConfig.epsilonSteps;
        planner->set_eps_step(plannerConfig.epsilonSteps);
        planner->set_initialsolution_eps(plannerConfig.initialEpsilon);

        solutionIds.clear();
        t_replan_start = std::chrono::steady_clock::now();
        env->setPlanningTimeout(t_replan_start, maxTime.toSeconds());
        bool replan_success = planner->replan(maxTime.toSeconds(), &solutionIds);
        t_replan_end = std::chrono::steady_clock::now();
        num_expands = planner->get_n_expands();
        final_epsilon = planner->get_final_epsilon();

        if(!replan_success)
        {
            double elapsed = std::chrono::duration<double>(t_replan_end - t_replan_start).count();
            if (elapsed >= maxTime.toSeconds() * 0.98)
            {
                LOG_WARN_S << "Planning failed due to TIMEOUT! Maximum time limit of "
                           << maxTime.toSeconds() << "s exceeded. State space expansions: " << num_expands;
                planning_res = TIMEOUT;
            }
            else
            {
                LOG_WARN_S << "Planning failed: NO SOLUTION EXISTS between start and goal after "
                           << num_expands << " expansions. The goal is unreachable or blocked by traversability constraints.";
                if (num_expands <= 1)
                {
                    LOG_WARN_S << "Note: Very few state space expansions (" << num_expands
                               << "). This typically indicates that all successor states from the start position are blocked. "
                               << "Ensure the start pose is not too close to obstacles, that corridorWidth is wide enough, and that minTurningRadius is appropriate.";
                }
                planning_res = NO_SOLUTION;
            }

            if(dumpOnError)
                PlannerDump dump(*this, "no_solution", maxTime, startbody2Mls, endbody2Mls);
        }
        else
        {
            LOG_DEBUG_S << "num expands: " << num_expands;
            LOG_DEBUG_S << "Epsilon is " << final_epsilon;

            std::vector<PlannerStats> stats;
            planner->get_search_stats(&stats);
            if(plannerConfig.useReedsSheppFinalPath)
            {
                env->getTrajectoryReedsShepp(solutionIds, resultTrajectory2D, true, start_translation, start_pose.getYaw(), goal_translation, end_pose.getYaw(), ground2Body,
                                             plannerConfig.reedsSheppStepSize, plannerConfig.reedsSheppMaxShortcut);
                env->getTrajectoryReedsShepp(solutionIds, resultTrajectory3D, false, start_translation, start_pose.getYaw(), goal_translation, end_pose.getYaw(), ground2Body,
                                             plannerConfig.reedsSheppStepSize, plannerConfig.reedsSheppMaxShortcut);
            }
            else
            {
                env->getTrajectory(solutionIds, resultTrajectory2D, true, start_translation, goal_translation, end_pose.getYaw(), ground2Body);
                env->getTrajectory(solutionIds, resultTrajectory3D, false, start_translation, goal_translation,end_pose.getYaw(), ground2Body);
            }
            t_trajectory_extraction = std::chrono::steady_clock::now();
            planning_res = FOUND_SOLUTION;
        }
    }
    catch(const SBPL_Exception& ex)
    {
        LOG_ERROR_S << "Caught sbpl exception: " << ex.what();
        if(dumpOnError)
            PlannerDump dump(*this, "no_solution", maxTime, startbody2Mls, endbody2Mls);
        planning_res = NO_SOLUTION;
    }

    if(dumpOnSuccess && planning_res == FOUND_SOLUTION)
        PlannerDump dump(*this, "success", maxTime, startbody2Mls, endbody2Mls);

    auto t_end_total = std::chrono::steady_clock::now();

    double d_env_init = std::chrono::duration<double>(t_env_init - t_start_total).count();
    double d_set_start = std::chrono::duration<double>(t_set_start - t_env_init).count();
    double d_set_goal = std::chrono::duration<double>(t_set_goal - t_set_start).count();
    double d_planner_setup = std::chrono::duration<double>(t_planner_setup - t_set_goal).count();
    double d_replan = std::chrono::duration<double>(t_replan_end - t_replan_start).count();
    double d_trajectory = 0.0;
    if (planning_res == FOUND_SOLUTION) {
        d_trajectory = std::chrono::duration<double>(t_trajectory_extraction - t_replan_end).count();
    }
    double d_total = std::chrono::duration<double>(t_end_total - t_start_total).count();

    LOG_INFO_S << "[KPI] --- PLANNING PERFORMANCE BREAKDOWN ---";
    LOG_INFO_S << "[KPI] Env Init:              " << d_env_init << "s";
    LOG_INFO_S << "[KPI] Set Start State:       " << d_set_start << "s";
    LOG_INFO_S << "[KPI] Set Goal State:        " << d_set_goal << "s (includes heuristic Dijkstra)";
    LOG_INFO_S << "[KPI] Planner Setup/Memory:  " << d_planner_setup << "s";
    LOG_INFO_S << "[KPI] Search/Replan (A*):    " << d_replan << "s";
    LOG_INFO_S << "[KPI] Trajectory Extraction: " << d_trajectory << "s";
    LOG_INFO_S << "[KPI] Total Planning Time:   " << d_total << "s";
    LOG_INFO_S << "[KPI] State space expands:   " << num_expands;
    LOG_INFO_S << "[KPI] Final Epsilon:         " << final_epsilon;
    LOG_INFO_S << "[KPI] ---------------------------------------";

    return planning_res;
}

std::vector< Motion > Planner::getMotions() const
{
    if (!env) {
        return {};
    }
    return env->getMotions(solutionIds);
}

const std::shared_ptr<const traversability_generator3d::TravMap3d > Planner::getTraversabilityMap() const
{
    if (!env) {
        return nullptr;
    }
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
    // One thread knob: when travgen runs under the planner, its expansion uses
    // the planner's thread count (0 would mean "do not parallelize").
    traversabilityConfig.numThreads = static_cast<int>(plannerConfig.numThreads);
    if(env){
        env->setTravConfig(traversabilityConfig);
    }
}

 void Planner::setPlannerConfig(const PlannerConfig& config)
 {
     plannerConfig = config;
     if(env){
         env->setCorridorWidth(config.corridorWidth);
         env->setGoalOrientationMargin(config.goalOrientationMargin);
         env->setGoalDistanceMargin(config.goalDistanceMargin);
     }
 }

}
