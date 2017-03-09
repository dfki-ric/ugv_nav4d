#pragma once
#include <maps/grid/MLSMap.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <boost/shared_ptr.hpp>
#include <base/Trajectory.hpp>
#include <motion_planning_libraries/sbpl/SbplMotionPrimitives.hpp>
#include <motion_planning_libraries/sbpl/SbplSplineMotionPrimitives.hpp>
#include "TraversabilityGenerator3d.hpp"
#include "PreComputedMotions.hpp"
#include "EnvironmentXYZTheta.hpp"

class ARAPlanner;

namespace ugv_nav4d
{

class Planner
{
    boost::shared_ptr<EnvironmentXYZTheta> env;
    boost::shared_ptr<ARAPlanner> planner;
    
    const motion_planning_libraries::SplinePrimitivesConfig splinePrimitiveConfig; 
    const motion_planning_libraries::Mobility mobility;
    TraversabilityConfig traversabilityConfig;
    
    std::vector<int> solution;
    
public:
    Planner(const motion_planning_libraries::SplinePrimitivesConfig &primitiveConfig, const TraversabilityConfig &traversabilityConfig,
            const motion_planning_libraries::Mobility& mobility);
    
    template <maps::grid::MLSConfig::update_model SurfacePatch>
    void updateMap(const maps::grid::MLSMap<SurfacePatch>& mls)
    {
        boost::shared_ptr<maps::grid::MultiLevelGridMap<maps::grid::SurfacePatchBase>> mlsPtr(getMLSBase(mls));

        if(!env)
        {
            env.reset(new EnvironmentXYZTheta(mlsPtr, traversabilityConfig, splinePrimitiveConfig, mobility));
        }
        else
        {
            env->updateMap(mlsPtr);
        }
    }
    
    /** @p maxTime Maximum processor time to use. */
    bool plan(const base::Time& maxTime, base::samples::RigidBodyState& start, base::samples::RigidBodyState& end);
    
    void getTrajectory(std::vector<base::Trajectory> &trajectory);
    
    std::vector<Motion> getMotions() const;
    
    void setTravConfig(const TraversabilityConfig& config);
    
    maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase* >getTraversabilityMap() const;
    
    boost::shared_ptr<EnvironmentXYZTheta> getEnv() const;

private:
    template <class mapType>
    boost::shared_ptr<maps::grid::MultiLevelGridMap<maps::grid::SurfacePatchBase>> getMLSBase(const mapType &map)
    {
        std::cout << "Grid has size " << map.getSize().transpose() << std::endl;

        maps::grid::MultiLevelGridMap<maps::grid::SurfacePatchBase> *mlsBase = new maps::grid::MultiLevelGridMap<maps::grid::SurfacePatchBase>(map);

        boost::shared_ptr<maps::grid::MultiLevelGridMap<maps::grid::SurfacePatchBase>> mlsPtr(mlsBase);

        return mlsPtr;
    }
};

}
