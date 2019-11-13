#pragma once
#include <maps/grid/MLSMap.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <boost/shared_ptr.hpp>
#include <sbpl_spline_primitives/SbplSplineMotionPrimitives.hpp>
#include "EnvironmentXYZTheta.hpp"

class ARAPlanner;

namespace ugv_nav4d
{

class PlannerDump;
    
class Planner
{
protected:
    friend class PlannerDump;
    typedef EnvironmentXYZTheta::MLGrid MLSBase;
    boost::shared_ptr<EnvironmentXYZTheta> env;
    boost::shared_ptr<ARAPlanner> planner;
    
    const sbpl_spline_primitives::SplinePrimitivesConfig splinePrimitiveConfig; 
    const Mobility mobility;
    TraversabilityConfig traversabilityConfig;
    std::vector<int> solutionIds;
    
    std::function<void ()> travMapCallback;
    
    /**are buffered and reused for a more robust map generation */
    std::vector<Eigen::Vector3d> previousStartPositions;
    
public:
    enum PLANNING_RESULT {
        GOAL_INVALID,
        START_INVALID, 
        NO_SOLUTION,
        NO_MAP,
        INTERNAL_ERROR,
        FOUND_SOLUTION,
    };
    
    Planner(const sbpl_spline_primitives::SplinePrimitivesConfig &primitiveConfig, const TraversabilityConfig &traversabilityConfig,
            const Mobility& mobility);
    
    template <maps::grid::MLSConfig::update_model SurfacePatch>
    void updateMap(const maps::grid::MLSMap<SurfacePatch>& mls)
    {
        std::shared_ptr<MLSBase> mlsPtr = std::make_shared<MLSBase>(mls);

        if(!env)
        {
            env.reset(new EnvironmentXYZTheta(mlsPtr, traversabilityConfig, splinePrimitiveConfig, mobility));
        }
        else
        {
            env->updateMap(mlsPtr);
        }
    }
    
    void updateMap(const MLSBase &mls)
    {
        std::shared_ptr<MLSBase> mlsPtr= std::make_shared<MLSBase>(mls);

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

    /**
     * This callback is executed, whenever a new traverability map
     * was expanded
     * */
    void setTravMapCallback(const std::function<void ()> &callback);
    
    std::vector<Motion> getMotions() const;
    
    /** Plan a path from @p start to @p end.
     * @param maxTime Maximum processor time to use.
     * @param startbody2Mls The start position of the body in mls coordinates. This should be the location of the body-frame.
     *                      The planner assumes that this location is config.distToGround meters above (!!!) the map. 
     *                      The planner will transform this location to the ground frame using config.distToGround.
     *                      If this location is not exactly config.distToGround meters above the map, the planner will
     *                      fail to find the patch that the robot is standing on.
     * @param endbody2Mls The goal position of the body in mls coordinates. See @p startbody2Mls for more details.
     * @param resultTrajectory2D The resulting trajectory without z-coordinates (all z-values are set to zero). 
     *                           This trajectory exists to avoid a bug in spline interpolation. In certain corner 
     *                           cases (when the z change between two steps is much greater than the xy change) 
     *                           the spline interpolator will fit an S-curve instead of a straight line between the 
     *                           two patches. This results in an increase-decrease-increase pattern in xy-direction
     *                           resulting in a robot motion that stutters. Setting the z-axis to zero was choosen as 
     *                           a fix because the trajectory follower (at the time of writing) ignores the z-axis anyway.
     *                           FIXME Further investigation is needed to find a real fix for this!
     *                           A ticket for this bug exists: https://git.hb.dfki.de/entern/ugv_nav4d/issues/1
     * @param resultTrajectory3D The resulting trajectory. Make sure to read the comment for @p resultTrajectory2D to understand
     *                           why this exists!
     * @param dumpOnError If true a planner dump will be written in case of error. This dump can be loaded and analyzed later
     *                    using the ugv_nav4d_replay tool.
     * */
    PLANNING_RESULT plan(const base::Time& maxTime, const base::samples::RigidBodyState& startbody2Mls,
                         const base::samples::RigidBodyState& endbody2Mls, std::vector<base::Trajectory>& resultTrajectory2D,
                         std::vector<base::Trajectory>& resultTrajectory3D, bool dumpOnError = false);

    void genTravMap(const base::samples::RigidBodyState& startbody2Mls);    
    
    void setTravConfig(const TraversabilityConfig& config);
    
    const maps::grid::TraversabilityMap3d<TravGenNode*> &getTraversabilityMap() const;
    
    boost::shared_ptr<EnvironmentXYZTheta> getEnv() const;

};

}
