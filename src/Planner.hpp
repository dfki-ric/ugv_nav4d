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
protected:
    typedef EnvironmentXYZTheta::MLGrid MLSBase;
    boost::shared_ptr<EnvironmentXYZTheta> env;
    boost::shared_ptr<ARAPlanner> planner;
    
    const motion_planning_libraries::SplinePrimitivesConfig splinePrimitiveConfig; 
    const motion_planning_libraries::Mobility mobility;
    TraversabilityConfig traversabilityConfig;
    std::vector<int> solutionIds;
    
public:
    Planner(const motion_planning_libraries::SplinePrimitivesConfig &primitiveConfig, const TraversabilityConfig &traversabilityConfig,
            const motion_planning_libraries::Mobility& mobility);
    
    template <maps::grid::MLSConfig::update_model SurfacePatch>
    void updateMap(const maps::grid::MLSMap<SurfacePatch>& mls)
    {
        boost::shared_ptr<MLSBase> mlsPtr(getMLSBase(mls));

        if(!env)
        {
            env.reset(new EnvironmentXYZTheta(mlsPtr, traversabilityConfig, splinePrimitiveConfig, mobility));
        }
        else
        {
            env->updateMap(mlsPtr);
        }
    }
    
    void setInitialPatch(const Eigen::Affine3d& body2Mls, double patchRadius);
    
    std::vector<Motion> getMotions() const;
    
    /** Plan a path from @p start to @p end.
     * @param maxTime Maximum processor time to use.
     * */
    bool plan(const base::Time& maxTime, const base::samples::RigidBodyState& start,
              const base::samples::RigidBodyState& end, std::vector<base::Trajectory>& resultTrajectory);
    
    
    void setTravConfig(const TraversabilityConfig& config);
    
    maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase* >getTraversabilityMap() const;
    
    boost::shared_ptr<EnvironmentXYZTheta> getEnv() const;
    
private:
    template <class mapType>
    boost::shared_ptr<MLSBase> getMLSBase(const mapType &map)
    {
        std::cout << "Grid has size " << map.getSize().transpose() << std::endl;

        boost::shared_ptr<MLSBase> mlsPtr(new MLSBase(map));

        return mlsPtr;
    }
};

}
