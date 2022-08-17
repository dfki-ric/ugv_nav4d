#pragma once
#include <unordered_map>
#include <base/Eigen.hpp>
#include <traversability_generator3d/TraversabilityConfig.hpp>

namespace maps { namespace grid 
{
class TraversabilityNodeBase;
}}

namespace ugv_nav4d
{

class traversability_generator3d::TraversabilityConfig;
    
class Dijkstra
{
public:
    
    /** Computes the heuristic cost from @p source to all reachable nodes
     *  @param outDistances mapping from node to distance
     *  @param maxDist the maximum possible distance*/
    static void computeCost(const maps::grid::TraversabilityNodeBase* source,
                            std::unordered_map<const maps::grid::TraversabilityNodeBase*, double> &outDistances,
                            const traversability_generator3d::TraversabilityConfig& config);

private:
    static double getHeuristicDistance(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                                       const traversability_generator3d::TraversabilityConfig& config);
};
    
} 