#include "Planner.hpp"
#include "EnvironmentXYZTheta.hpp"
#include <sbpl/planners/araplanner.h>
#include <sbpl/utils/mdpconfig.h>
#include <maps/grid/MultiLevelGridMap.hpp>

using namespace maps::grid;

Planner::Planner(const motion_planning_libraries::MotionPrimitivesConfig& primitiveConfig1, const TraversabilityGenerator3d::Config& traversabilityConfig) : 
    primitiveConfig(primitiveConfig1)
    , traversabilityConfig(traversabilityConfig)
{
    if(traversabilityConfig.gridResolution != primitiveConfig.mGridSize)
        throw std::runtime_error("Planner::Planner : Configuration error, grid resolution of Primitives and TraversabilityGenerator3d differ");
    
    primitives = new motion_planning_libraries::SbplMotionPrimitives(primitiveConfig);
    primitives->createPrimitives();
    std::cout << "Computed Primitives" << std::endl;

}


bool Planner::plan(base::samples::RigidBodyState& start, base::samples::RigidBodyState& end)
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
    
    planner->set_search_mode(true);

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

    if(!planner->replan(1.0, &solution))
        return false;
 
    std::vector<PlannerStats> stats;
    
    planner->get_search_stats(&stats);
    
    std::cout << std::endl << "Stats" << std::endl;
    for(const PlannerStats &s: stats)
    {
        std::cout << "cost " << s.cost << " time " << s.time << "num childs " << s.expands << std::endl;
    }
    
    return true;
}

void Planner::updateMap(const maps::grid::MLSMapSloped &mlsSloped)
{
    std::cout << "Grid has size " << mlsSloped.getSize().transpose() << std::endl;

    MultiLevelGridMap<SurfacePatchBase> *mlsBase = new MultiLevelGridMap<SurfacePatchBase>(mlsSloped.getNumCells(), mlsSloped.getResolution(), mlsSloped.getLocalMapData());

    std::cout << "MLSBase Resolutition " << mlsBase->getResolution().transpose() << std::endl;
    
    for(size_t y = 0 ; y < mlsSloped.getNumCells().y(); y++)
    {
        for(size_t x = 0 ; x < mlsSloped.getNumCells().x(); x++)
        {
            Index idx(x, y);
            for(auto &p : mlsSloped.at(idx))
            {
//                 std::cout << "Patch ! " << idx.transpose() << std::endl;
                SurfacePatchBase basePatch(p.getTop(), p.getTop() - p.getBottom());
                mlsBase->at(idx).insert(basePatch);
            }
        }
    }

    std::cout << "BaseGrid has size " << mlsBase->getSize().transpose() << std::endl;

    boost::shared_ptr<MultiLevelGridMap<SurfacePatchBase>> mlsPtr(mlsBase);

    if(!env) {
        env.reset(new EnvironmentXYZTheta(mlsPtr, traversabilityConfig, *primitives));
    }
    else
    {
        env->updateMap(mlsPtr);
    }
}

void Planner::getTrajectory(std::vector< base::Trajectory >& trajectory)
{
    if(solution.size() < 2)
        return;
    
    trajectory.clear();
    
    std::cout << "Solution: " << std::endl;
    size_t lastMotion = env->getMotion(solution[0], solution[1]).id;
    
    base::Trajectory curPart;
    
    std::vector<base::Vector3d> positions;
    
    for(size_t i = 0; i < solution.size() - 1; ++i)
    {
        const EnvironmentXYZTheta::Motion &curMotion(env->getMotion(solution[i], solution[i+1]));
        
        std::cout << "Motion has id " << curMotion.id << std::endl;
        
        if(lastMotion != curMotion.id)
        {
            curPart.spline.interpolate(positions);
            curPart.speed = 0.2;
            positions.clear();
            trajectory.push_back(curPart);
        }

        std::cout << solution[i] << " ";
        const maps::grid::Vector3d start = env->getStatePosition(solution[i]);
        std::cout << "Intermediate Poses : " << curMotion.intermediatePoses.size() << std::endl;
        for(const base::Pose2D& pose : curMotion.intermediatePoses)
        {
            //need to offset by start because the poses are relative to (0/0)
            positions.emplace_back(pose.position.x() + start.x(), pose.position.y() + start.y(), start.z());
            
            std::cout << "Intermediate position " << positions.back().transpose() << std::endl;
        }
    }
    std::cout << std::endl;

    curPart.spline.interpolate(positions);
    curPart.speed = 0;
    trajectory.push_back(curPart);
    
}

maps::grid::TraversabilityMap3d< TraversabilityNodeBase* > Planner::getTraversabilityMap() const
{
    return env->getTraversabilityMap();
}
