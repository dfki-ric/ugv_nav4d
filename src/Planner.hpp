#pragma once
#include <maps/grid/MLSMap.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <boost/shared_ptr.hpp>
#include <base/Trajectory.hpp>
#include <motion_planning_libraries/sbpl/SbplMotionPrimitives.hpp>
#include <trav_gen_3d/TraversabilityGenerator3d.hpp>

class EnvironmentXYZTheta;
class ARAPlanner;
class Planner
{
    boost::shared_ptr<EnvironmentXYZTheta> env;
    boost::shared_ptr<ARAPlanner> planner;
    
    const motion_planning_libraries::MotionPrimitivesConfig primitiveConfig; 
    const TraversabilityGenerator3d::Config traversabilityConfig;
    motion_planning_libraries::SbplMotionPrimitives *primitives;
    
    std::vector<int> solution;
    
    void updateHeight(base::Vector3d &pos) const;
    
public:
    Planner(const motion_planning_libraries::MotionPrimitivesConfig &primitiveConfig, const TraversabilityGenerator3d::Config &traversabilityConfig);
    
    void updateMap(const maps::grid::MLSMapSloped& mlsSloped);
    
    bool plan(base::samples::RigidBodyState &start, base::samples::RigidBodyState &end);
    
    void getTrajectory(std::vector<base::Trajectory> &trajectory);
    
    maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase* >getTraversabilityMap() const;
};


