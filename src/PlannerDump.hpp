#pragma once

#include <sbpl_spline_primitives/SplinePrimitivesConfig.hpp>
#include "TraversabilityConfig.hpp"
#include "TraversabilityGenerator3d.hpp"
#include "Config.hpp"
#include <base/samples/RigidBodyState.hpp>

namespace ugv_nav4d {

class Planner;
    
class PlannerDump
{
    sbpl_spline_primitives::SplinePrimitivesConfig splinePrimitiveConfig; 
    Mobility mobility;
    TraversabilityConfig traversabilityConfig;
    PlannerConfig plannerConfig;
    
    base::samples::RigidBodyState start;
    base::samples::RigidBodyState goal;
    base::Time maxTime;
    
    typedef TraversabilityGenerator3d::MLGrid MLSBase;
    std::string getUnusedFilename(const std::string& filePostfix) const;

    MLSBase mlsMap;
public:
    
    /**
     * Constructor for loading
     * */
    PlannerDump(const std::string &dumpName);

    PlannerDump(const ugv_nav4d::Planner& planner, const std::string& filePostfix, const base::Time& maxTime, const base::samples::RigidBodyState& startbody2Mls, const base::samples::RigidBodyState& endbody2Mls);
    
    const sbpl_spline_primitives::SplinePrimitivesConfig &getSplineConfig() const
    {
        return splinePrimitiveConfig;
    }

    const Mobility &getMobilityConf() const
    {
        return mobility;
    }
    
    const TraversabilityConfig &getTravConfig() const
    {
        return traversabilityConfig;
    }
    
    const PlannerConfig &getPlannerConfig() const
    {
        return plannerConfig;
    }
    
    const base::samples::RigidBodyState &getStart() const
    {
        return start;
    }
    const base::samples::RigidBodyState &getGoal() const
    {
        return goal;
    }
    const base::Time &getMaxTime() const
    {
        return maxTime;
    }
    const MLSBase &getMlsMap() const
    {
        return mlsMap;
    }

};

}
