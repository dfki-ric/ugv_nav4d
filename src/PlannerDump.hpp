#pragma once

#include <motion_planning_libraries/sbpl/SplinePrimitivesConfig.hpp>
#include <motion_planning_libraries/Config.hpp>
#include "TraversabilityConfig.hpp"
#include "TraversabilityGenerator3d.hpp"
#include <base/samples/RigidBodyState.hpp>

namespace ugv_nav4d {

class Planner;
    
class PlannerDump
{
    motion_planning_libraries::SplinePrimitivesConfig splinePrimitiveConfig; 
    motion_planning_libraries::Mobility mobility;
    TraversabilityConfig traversabilityConfig;
    
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
    
    const motion_planning_libraries::SplinePrimitivesConfig &getSplineConfig() const
    {
        return splinePrimitiveConfig;
    }

    const motion_planning_libraries::Mobility &getMobilityConf() const
    {
        return mobility;
    }
    
    const TraversabilityConfig &getTravConfig() const
    {
        return traversabilityConfig;
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
