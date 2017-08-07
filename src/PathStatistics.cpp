#include "PathStatistics.hpp"
#include <unordered_set>
#include <deque>

ugv_nav4d::PathStatistic::PathStatistic(const ugv_nav4d::TraversabilityConfig& config) : 
        obstacles(0), 
        frontiers(0),
        config(config)
{
    minDistance.resize(maps::grid::TraversabilityNodeBase::FRONTIER + 1, std::numeric_limits< double >::max());
    minDistToObstacle = std::numeric_limits< double >::max();
}


void ugv_nav4d::PathStatistic::updateStatistic(const maps::grid::TraversabilityNodeBase* node)
{
    switch(node->getType())
    {
        case maps::grid::TraversabilityNodeBase::TRAVERSABLE:
            break;
        case maps::grid::TraversabilityNodeBase::FRONTIER:
            frontiers++;
            break;
        default:
            minDistToObstacle = std::min(minDistance[node->getType()], minDistToObstacle);
            obstacles++;
            break;
    }
}

void ugv_nav4d::PathStatistic::calculateStatistics(std::vector< ugv_nav4d::TravGenNode* > path)
{
    //dist is in real world 
    const double neighborSquareDist = config.costFunctionDist * config.costFunctionDist;
    std::unordered_set<maps::grid::TraversabilityNodeBase*> neighbors; //all neighbors that are closer than neighborSquareDist

    const Eigen::Vector2d travGridResolution(config.gridResolution, config.gridResolution);
    
    //find all neighbors within corridor around path
    for(TravGenNode* node : path)
    {
        //vector is not the the most efficient when using std::find but for small vectors it should be ok.
        //linear serach on a cached vector is as fast as unordered_set lookup for vector sizes < 100
        // (yes, I benchmarked)
        std::deque<maps::grid::TraversabilityNodeBase*> nodes;
        std::unordered_set<maps::grid::TraversabilityNodeBase*> visited;
        nodes.push_back(node);
        const maps::grid::Vector2d nodePos = node->getIndex().cast<double>().cwiseProduct(travGridResolution);
        do
        {
            maps::grid::TraversabilityNodeBase* currentNode = nodes.front();
            nodes.pop_front();
            neighbors.insert(currentNode);
            
            for(auto neighbor : currentNode->getConnections())
            {
                //check if we have already visited this node (happens because double connected graph)
                if(visited.find(neighbor) != visited.end())
                    continue;

                visited.insert(neighbor);
                    
                //check if node is within corridor
                const maps::grid::Vector2d neighborPos = neighbor->getIndex().cast<double>().cwiseProduct(travGridResolution);
                double curSquaredNorm = (neighborPos - nodePos).squaredNorm();
                if(curSquaredNorm > neighborSquareDist)
                    continue;
                
                minDistance[neighbor->getType()] = std::min(curSquaredNorm, minDistance[neighbor->getType()]);
                nodes.push_back(neighbor);
            }
        }while(!nodes.empty());
    }
    
    //get real norm, not squared one
    for(double &d : minDistance)
    {
        d = sqrt(d);
    }
    
    for(maps::grid::TraversabilityNodeBase* n : neighbors)
    {
        updateStatistic(n);
    }   
}

double ugv_nav4d::PathStatistic::getMinDistToFrontiers() const
{
    return minDistance[maps::grid::TraversabilityNodeBase::FRONTIER];
}

double ugv_nav4d::PathStatistic::getMinDistToObstacles() const
{
    return minDistToObstacle;
}
