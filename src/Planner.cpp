#include "Planner.hpp"
#include "TravMapBfsVisitor.hpp"
#include <sbpl/planners/araplanner.h>
#include <sbpl/utils/mdpconfig.h>
#include <maps/grid/MultiLevelGridMap.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.h>
#include <vizkit3d_debug_drawings/DebugDrawingColors.h>
#include <base/Eigen.hpp>

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




bool Planner::plan(const base::Time& maxTime, const base::samples::RigidBodyState& start,
              const base::samples::RigidBodyState& end, std::vector<base::Trajectory>& resultTrajectory)
{
    if(!env)
        throw std::runtime_error("Planner::plan : Error : No map was set");
    
    resultTrajectory.clear();
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
    
    std::vector<int> solutionIds;
    if(!planner->replan(maxTime.toSeconds(), &solutionIds))
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
    
    env->getTrajectory(solutionIds, resultTrajectory);
    return true;
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

bool Planner::planToNextFrontier(const base::Time& maxTime, const base::samples::RigidBodyState& start,
                                 const base::Vector3d& closeTo, double goalOrientationZ,
                                 std::vector<base::Trajectory>& resultTrajectory)
{
    std::vector<const TravGenNode*> frontier;
    for(const maps::grid::LevelList<TravGenNode*>& list : env->getTraversabilityMap())
    {
        for(const TravGenNode* node : list)
        {
            if(node->getType() == TraversabilityNodeBase::FRONTIER)
            {
                frontier.emplace_back(node);
            }
        }
    }    
    
    if(frontier.size() <= 0)
    {
        std::cout << "No frontier patches found\n";
        return false;
    }

    const TravGenNode* nextFrontierNode = nullptr;
    base::Vector3d nextFrontierPos;
    double distToNextFrontierNode = std::numeric_limits<double>::max();
        
    for(const TravGenNode* frontierNode : frontier)
    {
        base::Vector3d pos(frontierNode->getIndex().x() * traversabilityConfig.gridResolution, 
                           frontierNode->getIndex().y() * traversabilityConfig.gridResolution,
                           frontierNode->getHeight());
        pos = env->getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
        
        const double distToFrontierNode = (pos - closeTo).norm();
        if(distToFrontierNode < distToNextFrontierNode)
        {
            nextFrontierNode = frontierNode;
            distToNextFrontierNode = distToFrontierNode;
            nextFrontierPos = pos;
        }
    }
    
    assert(nextFrontierNode != nullptr);
    
    DRAW_CYLINDER("frontier target", nextFrontierPos, base::Vector3d(0.2, 0.2, 1), vizkit3dDebugDrawings::Color::cyan);
       
    //search for a neighbor that passes the obstacle check. I.e. that we can acutally reach
    const TravGenNode* traversableNeighbor = nullptr;
    base::Vector3d traversableNeighborPos(0, 0, 0);
    TravMapBfsVisitor::visit(nextFrontierNode, 
        [&traversableNeighbor, &nextFrontierNode, &nextFrontierPos, this, &start, goalOrientationZ, &traversableNeighborPos]
        (const TravGenNode* currentNode, bool& visitChildren, bool& abort)
        {
            if((currentNode->getType() == maps::grid::TraversabilityNodeBase::TRAVERSABLE ||
               currentNode->getType() == TraversabilityNodeBase::FRONTIER) &&
               env->checkCollision(currentNode, goalOrientationZ))
            {
                traversableNeighbor = currentNode;
                
                base::Vector3d pos(currentNode->getIndex().x() * this->traversabilityConfig.gridResolution, 
                                   currentNode->getIndex().y() * this->traversabilityConfig.gridResolution,
                                   currentNode->getHeight());
                pos = this->env->getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
                traversableNeighborPos = pos;
                DRAW_CYLINDER("obstacleCheck", pos + base::Vector3d(traversabilityConfig.gridResolution / 2.0, traversabilityConfig.gridResolution / 2.0, traversabilityConfig.gridResolution / 2.0), base::Vector3d(0.05, 0.05, 2), vizkit3dDebugDrawings::Color::green);
                //found a nearby node that we can stand on, abort
                abort = true;
                
            }
            else
            {
                abort = false;
                base::Vector3d pos(currentNode->getIndex().x() * this->traversabilityConfig.gridResolution, 
                                   currentNode->getIndex().y() * this->traversabilityConfig.gridResolution,
                                   currentNode->getHeight());
                pos = this->env->getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
                
                DRAW_CYLINDER("obstacleCheck", pos + base::Vector3d(traversabilityConfig.gridResolution / 2.0, traversabilityConfig.gridResolution / 2.0, traversabilityConfig.gridResolution / 2.0), base::Vector3d(0.05, 0.05, 2), vizkit3dDebugDrawings::Color::red);
                
                const double dist = (nextFrontierPos - pos).norm();
                if(dist < 1)
                    visitChildren = true;
                else
                    visitChildren = false;
            }
        });
    
    base::samples::RigidBodyState goal;
    goal.position = traversableNeighborPos;
    //FIXME use goal orientation
    goal.orientation = start.orientation;
    
    
    return plan(maxTime, start, goal, resultTrajectory);
    
}

}
