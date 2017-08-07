#pragma once

#include "TravGenNode.hpp"
#include "TraversabilityConfig.hpp"

namespace ugv_nav4d
{
class PathStatistic
{
    size_t obstacles;
    double minDistToObstacle;
    size_t frontiers;
    std::vector<double> minDistance;
    const TraversabilityConfig &config;
public:
    PathStatistic(const TraversabilityConfig &config);
    
    void calculateStatistics(std::vector<TravGenNode*> path);
    
    void updateStatistic(const maps::grid::TraversabilityNodeBase* node);
    
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
};

}
