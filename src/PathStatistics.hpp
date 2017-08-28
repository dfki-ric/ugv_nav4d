#pragma once

#include "TravGenNode.hpp"
#include "TraversabilityConfig.hpp"
#include <base/Pose.hpp>

namespace ugv_nav4d
{
class PathStatistic
{
    const TraversabilityConfig &config;
public:
    class Stats
    {
        size_t obstacles;
        double minDistToObstacle;
        size_t frontiers;
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
    Stats robotStats;
    Stats boundaryStats;
public:
    
    PathStatistic(const TraversabilityConfig &config);
    
    void calculateStatistics(const std::vector<const TravGenNode*> &path, const std::vector<base::Pose2D> &poses, 
                             const maps::grid::TraversabilityMap3d<TravGenNode *> &trMap, const std::string &debugObstacleName = std::string());
    
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
