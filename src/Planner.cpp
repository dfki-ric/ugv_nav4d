#include "Planner.hpp"
#include <sbpl/planners/araplanner.h>
#include <sbpl/utils/mdpconfig.h>
#include <maps/grid/MultiLevelGridMap.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.h>
#include <vizkit3d_debug_drawings/DebugDrawingColors.h>

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




bool Planner::plan(const base::Time &maxTime, base::samples::RigidBodyState& start, base::samples::RigidBodyState& end)
{
    solution.clear();
    
    if(!env)
        throw std::runtime_error("Planner::plan : Error : No map was set");
    
    env->clear();
    
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
    planner->force_planning_from_scratch_and_free_memory();
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
    {
        std::cout << "num expands: " << planner->get_n_expands() << std::endl;
        return false;
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
    
    return true;
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

void Planner::setTravConfig(const TraversabilityConfig& config)
{
    if(config.gridResolution != splinePrimitiveConfig.gridSize)
        throw std::runtime_error("Planner::Planner : Configuration error, grid resolution of Primitives and TraversabilityGenerator3d differ");
    
    traversabilityConfig = config;
    if(env)
        env->setTravConfig(config);
}

void Planner::planShortestExplorationPath(const base::Vector3d& start) const
{
    std::cout << "started shortestpathPlanning" << std::endl;
    //FIXME very naive implementation, can probably be improved
    std::vector<const TravGenNode*> path;
    std::unordered_set<const TravGenNode*> frontier;
    for(const maps::grid::LevelList<TravGenNode*>& list : env->getTraversabilityMap())
    {
        for(const TravGenNode* node : list)
        {
            if(node->getType() == TraversabilityNodeBase::FRONTIER)
            {
                frontier.insert(node);
            }
        }
    }
    
    //FIXME assert that start node is not a frontier node
    
    const TravGenNode* currentNode = env->getTravGen().generateStartNode(start);
    path.push_back(currentNode);
    while(frontier.size() > 0)
    {
        std::cout << "frontier size "<< frontier.size() << std::endl;
        //find closest frontier node
        std::vector<double> distances;
        env->dijkstraComputeCost(currentNode, distances, 999999);//FIXME 99999999
        
        double closestDist = 99999999;//FIXME 99999999
        const TravGenNode* closestNode = nullptr;
        for(const TravGenNode* frontierNode : frontier)
        {
            const int id = frontierNode->getUserData().id;
            const double dist = distances[id];
            if(dist < closestDist)
            {
                closestDist = dist;
                closestNode = frontierNode;
            }
        }
        
        assert(nullptr != closestNode);
        frontier.erase(closestNode);
        path.push_back(closestNode);
        
        COMPLEX_DRAWING(
            Eigen::Vector3d start(currentNode->getIndex().x() * traversabilityConfig.gridResolution, currentNode->getIndex().y() * traversabilityConfig.gridResolution, currentNode->getHeight());
            start = env->getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * start;
            start.z() += 0.1;
            
            Eigen::Vector3d end(closestNode->getIndex().x() * traversabilityConfig.gridResolution, closestNode->getIndex().y() * traversabilityConfig.gridResolution, closestNode->getHeight());
            end = env->getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * end;
            end.z() += 0.1;
            
            DRAW_TEXT("shortest dist", start + (end - start)/2.0, std::to_string(distances[closestNode->getUserData().id]), 0.1, vizkit3dDebugDrawings::Color::magenta);
            
            DRAW_LINE("shortest exploration path", start, end, vizkit3dDebugDrawings::Color::magenta);
        );
        
        currentNode = closestNode;
    }
    std::cout << "done shortestpathPlanning" << std::endl;
    
}



}
