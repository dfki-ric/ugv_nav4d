#pragma once
#include <maps/grid/MLSMap.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <boost/shared_ptr.hpp>
#include <base/Trajectory.hpp>
#include <motion_planning_libraries/sbpl/SbplMotionPrimitives.hpp>
#include <motion_planning_libraries/sbpl/SbplSplineMotionPrimitives.hpp>
#include "TraversabilityGenerator3d.hpp"
#include "PreComputedMotions.hpp"

class ARAPlanner;

namespace ugv_nav4d
{

class EnvironmentXYZTheta;

class Planner
{
    boost::shared_ptr<EnvironmentXYZTheta> env;
    boost::shared_ptr<ARAPlanner> planner;
    
    const motion_planning_libraries::SplinePrimitivesConfig splinePrimitiveConfig; 
    const motion_planning_libraries::Mobility mobility;
    const TraversabilityConfig traversabilityConfig;
    
    std::vector<int> solution;
    
public:
    Planner(const motion_planning_libraries::SplinePrimitivesConfig &primitiveConfig, const TraversabilityConfig &traversabilityConfig,
            const motion_planning_libraries::Mobility& mobility);
    
    
    void updateMap(const maps::grid::MLSMapPrecalculated& mlsSloped);
    
    bool plan(const base::Time& maxTime, base::samples::RigidBodyState& start, base::samples::RigidBodyState& end);
    
    void getTrajectory(std::vector<base::Trajectory> &trajectory);
    
    std::vector<Motion> getMotions() const;
    
    maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase* >getTraversabilityMap() const;
    
    boost::shared_ptr<EnvironmentXYZTheta> getEnv() const;
};

}
