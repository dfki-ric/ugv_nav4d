#include "Planner.hpp"
#include "EnvironmentXYZTheta.hpp"
#include <sbpl/planners/araplanner.h>
#include <sbpl/utils/mdpconfig.h>
#include <maps/grid/MultiLevelGridMap.hpp>

using namespace maps::grid;

namespace ugv_nav4d
{


Planner::Planner(const motion_planning_libraries::SplinePrimitivesConfig& primitiveConfig, const TraversabilityConfig& traversabilityConfig,
                const motion_planning_libraries::Mobility& mobility) :
    splinePrimitiveConfig(primitiveConfig),
    mobility(mobility),
    traversabilityConfig(traversabilityConfig)
{
    if(traversabilityConfig.gridResolution != primitiveConfig.gridSize)
        throw std::runtime_error("Planner::Planner : Configuration error, grid resolution of Primitives and TraversabilityGenerator3d differ");
}



bool Planner::plan(const base::Time &maxTime, base::samples::RigidBodyState& start, base::samples::RigidBodyState& end)
{
    solution.clear();
    
    if(!env)
        throw std::runtime_error("Planner::plan : Error : No map was set");
    
    try {
        env->setStart(start.position, start.getYaw());
    } catch (std::runtime_error &e)
    {
        std::cout << e.what() << std::endl;
        return false;
    }
    
    try {
        env->setGoal(end.position, end.getYaw());
    } catch (std::runtime_error &e)
    {
        std::cout << e.what() << std::endl;
        return false;
    }
    
    if(!planner)
        planner.reset(new ARAPlanner(env.get(), true));
    
    planner->set_search_mode(false);

    MDPConfig mdp_cfg;
        
    if (! env->InitializeMDPCfg(&mdp_cfg)) {
        std::cout << "InitializeMDPCfg failed, start and goal id cannot be requested yet" << std::endl;
        return false;
    }
        
    std::cout << "SBPL: About to set start and goal, startid" << mdp_cfg.startstateid << std::endl;
    if (planner->set_start(mdp_cfg.startstateid) == 0) {
        std::cout << "Failed to set start state" << std::endl;
        return false;
    }

    if (planner->set_goal(mdp_cfg.goalstateid) == 0) {
        std::cout << "Failed to set goal state" << std::endl;
        return false;
    }

    planner->set_eps_step(0.5);
    
    if(!planner->replan(maxTime.toSeconds(), &solution))
        return false;
        
    std::cout << "Epsilon is " << planner->get_final_epsilon() << std::endl;

    std::vector<PlannerStats> stats;
    
    planner->get_search_stats(&stats);
    
    std::cout << std::endl << "Stats" << std::endl;
    for(const PlannerStats &s: stats)
    {
        std::cout << "cost " << s.cost << " time " << s.time << "num childs " << s.expands << std::endl;
    }
    
    return true;
}

template <class mapType>
boost::shared_ptr<MultiLevelGridMap<SurfacePatchBase>> getMLSBase(const mapType &map)
{
    std::cout << "Grid has size " << map.getSize().transpose() << std::endl;

    MultiLevelGridMap<SurfacePatchBase> *mlsBase = new MultiLevelGridMap<SurfacePatchBase>(map);


    boost::shared_ptr<MultiLevelGridMap<SurfacePatchBase>> mlsPtr(mlsBase);
    
    return mlsPtr;
}


void Planner::updateMap(const maps::grid::MLSMapPrecalculated &mlsSloped)
{
    boost::shared_ptr<MultiLevelGridMap<SurfacePatchBase>> mlsPtr(getMLSBase(mlsSloped));

    if(!env) {
        env.reset(new EnvironmentXYZTheta(mlsPtr, traversabilityConfig, splinePrimitiveConfig, mobility));
    }
    else
    {
        env->updateMap(mlsPtr);
    }
}

void Planner::getTrajectory(std::vector< base::Trajectory >& trajectory)
{
    return env->getTrajectory(solution, trajectory);
}

maps::grid::TraversabilityMap3d< TraversabilityNodeBase* > Planner::getTraversabilityMap() const
{
    return env->getTraversabilityBaseMap();
}

boost::shared_ptr< EnvironmentXYZTheta > Planner::getEnv() const
{
    return env;
}

std::vector<Motion> Planner::getMotions() const
{
    return env->getMotions(solution);
}



}
