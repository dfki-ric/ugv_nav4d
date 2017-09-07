#include "Planner.hpp"
#include <sbpl/planners/araplanner.h>
#include <sbpl/utils/mdpconfig.h>
#include <maps/grid/MultiLevelGridMap.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.h>
#include <vizkit3d_debug_drawings/DebugDrawingColors.h>
#include <base/Eigen.hpp>
#include "PlannerDump.hpp"

using namespace maps::grid;

namespace ugv_nav4d
{


Planner::Planner(const motion_planning_libraries::SplinePrimitivesConfig& primitiveConfig, const TraversabilityConfig& traversabilityConfig,
                const motion_planning_libraries::Mobility& mobility) :
    splinePrimitiveConfig(primitiveConfig),
    mobility(mobility)
{
    setTravConfig(traversabilityConfig);
}

void Planner::setInitialPatch(const Eigen::Affine3d& body2Mls, double patchRadius)
{
    Eigen::Affine3d ground2Body(Eigen::Affine3d::Identity());
    ground2Body.translation() = Eigen::Vector3d(0, 0, -traversabilityConfig.distToGround);
    
    env->setInitialPatch(body2Mls * ground2Body , patchRadius);
}

void Planner::setTravMapCallback(const std::function< void ()>& callback)
{
    travMapCallback = callback;
}

Planner::PLANNING_RESULT Planner::plan(const base::Time& maxTime, const base::samples::RigidBodyState& startbody2Mls, const base::samples::RigidBodyState& endbody2Mls, std::vector< base::Trajectory >& resultTrajectory, bool dumpOnError)
{
    
    CLEAR_DRAWING("successors");
    
    if(!env)
    {
        std::cout << "Planner::plan : Error : No map was set" << std::endl;
        return NO_MAP;
    }
    
    resultTrajectory.clear();
    env->clear();
    
    Eigen::Affine3d ground2Body(Eigen::Affine3d::Identity());
    ground2Body.translation() = Eigen::Vector3d(0, 0, -traversabilityConfig.distToGround);
    
    const Eigen::Affine3d startGround2Mls(startbody2Mls.getTransform() * ground2Body);
    const Eigen::Affine3d endGround2Mls(endbody2Mls.getTransform() * ground2Body);
    
    
    std::vector<base::Trajectory> moveOutOfObstacleTrajectory;
    
    try
    {
        env->setStart(startGround2Mls.translation(), base::getYaw(Eigen::Quaterniond(startGround2Mls.linear())));
    }
    catch(const ugv_nav4d::ObstacleCheckFailed& ex)
    {
        //Try to find the path of least resistance out of the obstacle
        base::Vector3d newStart;
        double newStartTheta;
        const base::Trajectory traj = env->findTrajectoryOutOfObstacle(startGround2Mls.translation(),
                                                                       base::getYaw(Eigen::Quaterniond(startGround2Mls.linear())),
                                                                       ground2Body, newStart, newStartTheta);
        if(traj.attributes.size() > 0)
        {
            moveOutOfObstacleTrajectory.push_back(traj);
            
            try
            {
                env->setStart(newStart, newStartTheta);
            }
            catch(const std::runtime_error& ex)
            {
                //new start is also not valid for some reason
                std::cout << "Tried to move start out of obstacle but failed\n";
                return NO_SOLUTION;
            }
            
        }
        else
        {
            //no way out of obstacle
            return NO_SOLUTION;
        }
    }
    catch(const std::runtime_error& ex)
    {
        if(travMapCallback)
            travMapCallback();

        if(dumpOnError)
            PlannerDump dump(*this, "bad_start", maxTime, startbody2Mls, endbody2Mls);

        return START_INVALID;
    }
    
    try
    {
        env->setGoal(endGround2Mls.translation(), base::getYaw(Eigen::Quaterniond(endGround2Mls.linear())));
    }
    catch(const std::runtime_error& ex)
    {
        if(dumpOnError)
            PlannerDump dump(*this, "bad_goal", maxTime, startbody2Mls, endbody2Mls);
        return GOAL_INVALID;
    }
    
    
    if(!planner)
        planner.reset(new ARAPlanner(env.get(), true));
    planner->force_planning_from_scratch_and_free_memory();
    planner->set_search_mode(false);

    MDPConfig mdp_cfg;
        
    if (! env->InitializeMDPCfg(&mdp_cfg)) {
        std::cout << "InitializeMDPCfg failed, start and goal id cannot be requested yet" << std::endl;
        return INTERNAL_ERROR;
    }
        
    std::cout << "SBPL: About to set start and goal, startid=" << mdp_cfg.startstateid << std::endl;
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
        planner->set_eps_step(1.0);
        planner->set_initialsolution_eps(10.0);
        
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
        
        env->getTrajectory(solutionIds, resultTrajectory, ground2Body);
    }
    catch(const SBPL_Exception& ex)
    {
        std::cout << "caught sbpl exception: " << ex.what() << std::endl;
        std::cout << "dumping state" << std::endl;
        if(dumpOnError)
            PlannerDump dump(*this, "no_solution", maxTime, startbody2Mls, endbody2Mls);
        return NO_SOLUTION;
    }
    
    
    //have to move out of obstacle before the trajectory can be executed
    if(moveOutOfObstacleTrajectory.size() > 0)
    {
        resultTrajectory.insert(resultTrajectory.begin(), moveOutOfObstacleTrajectory.begin(), moveOutOfObstacleTrajectory.end());
    }
    return FOUND_SOLUTION;
}

std::vector< Motion > Planner::getMotions() const
{
    return env->getMotions(solutionIds);
}

maps::grid::TraversabilityMap3d< TraversabilityNodeBase* > Planner::getTraversabilityMap() const
{
    return env->getTraversabilityBaseMap();
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

}
