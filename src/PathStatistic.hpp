#pragma once

#include <traversability_generator3d/TravGenNode.hpp>
#include <traversability_generator3d/TraversabilityConfig.hpp>
#include <base/Pose.hpp>

namespace ugv_nav4d
{
    
/** Calculates statistics about different patches encountered while
*   traversing a path.
*   I.e. counts patchtes that the robot will traverase when following a path */
class PathStatistic
{
    const traversability_generator3d::TraversabilityConfig &config;
public:
    
    class Stats
    {
        size_t obstacles;
        double minDistToObstacle;
        size_t frontiers;
        
        /** Minimum distance to each patch type.
         *  Indexed by patch type. */
        std::vector<double> minDistance;
    public:
        Stats();
        
        size_t getNumObstacles() const
        {
            return obstacles;
        };
        
        double getMinDistToObstacles() const;
        
        size_t getNumFrontiers() const
        {
            return frontiers;
        };    
        double getMinDistToFrontiers() const;
        void updateStatistic(const maps::grid::TraversabilityNodeBase* node);
        void updateDistance(const maps::grid::TraversabilityNodeBase* node, double distance);
    };

protected:
    /** Statistics about patches that the robot traverses while following a given path */
    Stats robotStats;
    /** Statistics about patches in the corridor outside the robot bounding box */
    Stats boundaryStats;
public:
    
    PathStatistic(const traversability_generator3d::TraversabilityConfig &config);
    
    /**
     * Accumulates statistics about all patches that are inside a config.costFunctionDist wide corridor around @p path.
     * This can be used to answer questions such as:
     *  - How many obstacles does the robot traverse when following this path
     *  - How many obstacles are close to the robot (but are not directly traversed) when following this path (boundaryStats.getNumObstacles())
     *  - How many frontiers does the robot traverse when following this path (robotStats.getNumFrontiers())
     *  - Is it safe to follow this path? (robotStats.getNumObstacles() == 0?)
     *  - What is the minimum distance to an obstacle on this path?
     *  - etc.
     * 
     * The statistics are used extensively during planning.
     * 
     * @param path List of patches that define the path.
     * @param poses List of poses on the path. There should be one pose for each patch.
     * @param debugObstacleName optional name of the debug drawing that should be used for this path
     */
    void calculateStatistics(const std::vector<const traversability_generator3d::TravGenNode*> &path, 
                             const std::vector<base::Pose2D> &poses, 
                             const traversability_generator3d::TravMap3d& trMap, 
                             const std::string &debugObstacleName = std::string());

    bool isPathFeasible(const std::vector<const traversability_generator3d::TravGenNode*> &path, 
                        const std::vector<base::Pose2D> &poses, 
                        const traversability_generator3d::TravMap3d& trMap);                             
    
    const Stats &getRobotStats() const
    {
        return robotStats;
    }

    const Stats &getBoundaryStats() const
    {
        return boundaryStats;
    }
    
};

}
