#include "Planner.hpp"
#include <sbpl/planners/araplanner.h>
#include <sbpl/utils/mdpconfig.h>
#include <maps/grid/MultiLevelGridMap.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>
#include <vizkit3d_debug_drawings/DebugDrawingColors.hpp>
#include <base/Eigen.hpp>
#include "PlannerDump.hpp"
#include <omp.h>
#include <cmath>
#include "Logger.hpp"

using namespace maps::grid;
using trajectory_follower::SubTrajectory;

namespace ugv_nav4d
{


Planner::Planner(const sbpl_spline_primitives::SplinePrimitivesConfig& primitiveConfig, const TraversabilityConfig& traversabilityConfig, 
        const Mobility& mobility, const PlannerConfig& plannerConfig) :
    splinePrimitiveConfig(primitiveConfig),
    mobility(mobility),
    plannerConfig(plannerConfig),
    mls2Ground(Eigen::Affine3d::Identity())
{
    setTravConfig(traversabilityConfig);
}


Planner::Planner(const sbpl_spline_primitives::SplinePrimitivesConfig& primitiveConfig, const TraversabilityConfig& traversabilityConfig, 
        const Mobility& mobility, const PlannerConfig& plannerConfig, const Eigen::Affine3d& mls2Ground) :
    splinePrimitiveConfig(primitiveConfig),
    mobility(mobility),
    plannerConfig(plannerConfig),
    mls2Ground(mls2Ground)
{
    setTravConfig(traversabilityConfig);
}

void Planner::setInitialPatch(const Eigen::Affine3d& body2Mls, double patchRadius)
{
    Eigen::Affine3d ground2Body(Eigen::Affine3d::Identity());
    ground2Body.translation() = Eigen::Vector3d(0, 0, -traversabilityConfig.distToGround);
    
    env->setInitialPatch(mls2Ground * body2Mls * ground2Body , patchRadius);
}

void Planner::setTravMapCallback(const std::function< void ()>& callback)
{
    travMapCallback = callback;
}

void Planner::genTravMap(const base::samples::RigidBodyState& start_pose)
{
    if(!env)
    {
        std::cout << "Planner::genTravMap : Error : No map was set" << std::endl;
        return;
    }
    
    env->clear();
 
    Eigen::Affine3d ground2Body(Eigen::Affine3d::Identity());
    ground2Body.translation() = Eigen::Vector3d(0, 0, -traversabilityConfig.distToGround);
    
    base::samples::RigidBodyState startbody2Mls = start_pose;
    startbody2Mls.setTransform(mls2Ground * startbody2Mls.getTransform());

    const Eigen::Affine3d startGround2Mls(startbody2Mls.getTransform() * ground2Body);
    
    previousStartPositions.push_back(startGround2Mls.translation());
    
    env->expandMap(previousStartPositions);

    if(travMapCallback)
        travMapCallback();
}


bool Planner::calculateGoal(const Eigen::Vector3d& start_translation, Eigen::Vector3d& translation, const double yaw) noexcept
{
    static constexpr double theta_step = EIGEN_PI / 10.;
    if(mobility.searchRadius < std::numeric_limits<double>::epsilon()) {
        return tryGoal(translation, yaw);
    } else {
        bool is_invalid = true;
        const double start_angle = std::atan2(start_translation.y() - translation.y(), start_translation.x() - translation.x());
        LOG_PLAN("start_angle", start_angle);
        LOG_PLAN("goal", translation);

        double current_radius = mobility.searchProgressSteps;
        double theta = start_angle;
        double theta_pi = 0;
        int multiplier = 1;
        Eigen::Vector2d pos(0, 0);
        while(is_invalid) {
            LOG_PLAN("translation change", pos.x(), pos.y());
            Eigen::Vector3d temp = translation;
            temp.x() += pos.x();
            temp.y() += pos.y();
            temp.z() = [&]{
                double z;
                if(env->getMlsMap().getClosestSurfacePos(temp, z)) {
                    return z;
                }
                return temp.z();
            }();
            
            if(tryGoal(temp, yaw)) {
                translation = temp; // for future use by calling function
                return true; 
            }

           
            // if the circle is completed and still no valid goal is found, increase the radius and try again
            if(std::abs(theta_pi - EIGEN_PI) < std::numeric_limits<double>::epsilon()) {
                current_radius += mobility.searchProgressSteps;
                theta_pi = 0;
                LOG_PLAN("reset theta", theta_pi);

                // do we have reached our max. search radius?
                if(current_radius > mobility.searchRadius) {
                    return false;
                }
            } 
            // only add a new value if we are positive, so we can explore on both sides.
            if(multiplier == 1) {
                theta_pi += theta_step;
                theta = std::remainder(start_angle + theta_pi, 2 * EIGEN_PI);
                multiplier = -1;
            } else {
                theta = std::remainder(start_angle - theta_pi, 2 * EIGEN_PI);
                multiplier = 1;
            }
            LOG_PLAN("Calc pos", current_radius, theta);
            // calculate new position
            pos.y() = current_radius * std::sin(theta);
            pos.x() = current_radius * std::cos(theta);
        }
    }
    return false;
}


bool Planner::tryGoal(const Eigen::Vector3d& translation, const double yaw) noexcept
{
    try
    {
        env->setGoal(translation, yaw);
    }
    catch(const std::runtime_error& ex)
    {
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
    
    std::cout << "Planning with " << plannerConfig.numThreads << " threads" << std::endl;
    omp_set_num_threads(plannerConfig.numThreads);
    
    
    V3DD::CLEAR_DRAWING("ugv_nav4d_successors");
    
    if(!env)
    {
        std::cout << "Planner::plan : Error : No map was set" << std::endl;
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

    startbody2Mls.setTransform(mls2Ground * startbody2Mls.getTransform());
    endbody2Mls.setTransform(mls2Ground * endbody2Mls.getTransform());

    std::cout << "start_pose position (raw): " << start_pose.position.transpose() << std::endl;
    std::cout << "end_pose position (raw): " << end_pose.position.transpose() << std::endl;

    const Eigen::Affine3d startGround2Mls(startbody2Mls.getTransform() * ground2Body);
    const Eigen::Affine3d endGround2Mls(endbody2Mls.getTransform() *ground2Body);
    
    startbody2Mls.setTransform(startGround2Mls);
    endbody2Mls.setTransform(endGround2Mls);

    //TODO maybe use a deque and limit to last 30 starts?
    previousStartPositions.push_back(startGround2Mls.translation());
    
    env->expandMap(previousStartPositions);
    if(travMapCallback)
        travMapCallback();
    try
    {
        env->setStart(startGround2Mls.translation(), base::getYaw(Eigen::Quaterniond(startGround2Mls.linear())));
    }
    catch(const ugv_nav4d::ObstacleCheckFailed& ex)
    {
        std::cout << "Start inside obstacle." << std::endl;
        if(dumpOnError)
            PlannerDump dump(*this, "start_inside_obstacle", maxTime, startbody2Mls, endbody2Mls);
        return START_INVALID;
    }
    catch(const std::runtime_error& ex)
    {
        if(dumpOnError)
            PlannerDump dump(*this, "bad_start", maxTime, startbody2Mls, endbody2Mls);
        return START_INVALID;
    }
    
    Eigen::Vector3d goal_translation = endGround2Mls.translation();
    if(!calculateGoal(startGround2Mls.translation(), goal_translation, base::getYaw(Eigen::Quaterniond(endGround2Mls.linear())))) {
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
        planner->set_search_mode(false);
    }
    catch(const SBPL_Exception& ex)
    {
        std::cout << "caught sbpl exception: " << ex.what() << std::endl;
        return NO_SOLUTION;
    }

    
    MDPConfig mdp_cfg;
        
    if (!env->InitializeMDPCfg(&mdp_cfg)) {
        std::cout << "InitializeMDPCfg failed, start and goal id cannot be requested yet" << std::endl;
        return INTERNAL_ERROR;
    }
    if (planner->set_start(mdp_cfg.startstateid) == 0) {
        std::cout << "Failed to set start state" << std::endl;
        return INTERNAL_ERROR;
    }
    if (planner->set_goal(mdp_cfg.goalstateid) == 0) {
        std::cout << "Failed to set goal state" << std::endl;
        return INTERNAL_ERROR;
    }

    try
    {
        std::cout << "Initial Epsilon: " << plannerConfig.initialEpsilon << ", steps: " << plannerConfig.epsilonSteps << std::endl;
        planner->set_eps_step(plannerConfig.epsilonSteps);
        planner->set_initialsolution_eps(plannerConfig.initialEpsilon);
        
        solutionIds.clear();
        if(!planner->replan(maxTime.toSeconds(), &solutionIds))
        {
            std::cout << "num expands: " << planner->get_n_expands() << std::endl;
            if(dumpOnError)
                PlannerDump dump(*this, "no_solution", maxTime, startbody2Mls, endbody2Mls);
            return NO_SOLUTION;
        }
            
        std::cout << "num expands: " << planner->get_n_expands() << std::endl;
        std::cout << "Epsilon is " << planner->get_final_epsilon() << std::endl;

        std::vector<PlannerStats> stats;
        
        planner->get_search_stats(&stats);
        
        std::cout << std::endl << "Stats" << std::endl;
        for(const PlannerStats &s: stats)
        {
            std::cout << "cost " << s.cost << " time " << s.time << "num childs " << s.expands << std::endl;
        }
        
        env->getTrajectory(solutionIds, resultTrajectory2D, true, mls2Ground * ground2Body);
        env->getTrajectory(solutionIds, resultTrajectory3D, false, mls2Ground * ground2Body);
    }
    catch(const SBPL_Exception& ex)
    {
        std::cout << "caught sbpl exception: " << ex.what() << std::endl;
        std::cout << "dumping state" << std::endl;
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

const maps::grid::TraversabilityMap3d<TravGenNode*> &Planner::getTraversabilityMap() const
{
    return env->getTraversabilityMap();
}

boost::shared_ptr< EnvironmentXYZTheta > Planner::getEnv() const
{
    return env;
}

void Planner::setTravConfig(const TraversabilityConfig& config)
{
    if(config.gridResolution != splinePrimitiveConfig.gridSize)
        throw std::runtime_error("Planner::Planner : Configuration error, grid resolution of Primitives and TraversabilityGenerator3d differ");
    
    traversabilityConfig = config;
    if(env)
        env->setTravConfig(config);
}

 void Planner::setPlannerConfig(const PlannerConfig& config)
 {
     plannerConfig = config;
 }

}
