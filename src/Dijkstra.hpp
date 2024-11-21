#pragma once
#include <unordered_map>
#include <base/Eigen.hpp>

namespace maps { namespace grid 
{
class TraversabilityNodeBase;
}}

namespace traversability_generator3d{
    class TraversabilityConfig;
}

namespace ugv_nav4d
{
class Dijkstra
{
public:
    
    /** Computes the heuristic cost from @p source to all reachable nodes
     *  @param outDistances mapping from node to distance
     *  @param maxDist the maximum possible distance*/
    static void computeCost(const maps::grid::TraversabilityNodeBase* source,
                            std::unordered_map<const maps::grid::TraversabilityNodeBase*, double> &outDistances,
                            const traversability_generator3d::TraversabilityConfig& config);
};
    
} 