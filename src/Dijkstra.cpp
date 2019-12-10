#include "Dijkstra.hpp"
#include "TraversabilityConfig.hpp"
#include <maps/grid/TraversabilityMap3d.hpp>

using namespace maps::grid;

namespace ugv_nav4d
{

    
void Dijkstra::computeCost(const TraversabilityNodeBase* source,
                           std::unordered_map<const TraversabilityNodeBase*, double>& outDistances,
                           const TraversabilityConfig& config)
{

    
    outDistances.clear();
    outDistances[source] = 0.0;
    
    std::set<std::pair<double, const TraversabilityNodeBase*>> vertexQ;
    vertexQ.insert(std::make_pair(outDistances[source], source));
   
    while (!vertexQ.empty()) 
    {
        const double dist = vertexQ.begin()->first;
        const TraversabilityNodeBase* u = vertexQ.begin()->second;
        
        vertexQ.erase(vertexQ.begin());
        
        const Eigen::Vector3d uPos(u->getIndex().x() * config.gridResolution,
                                   u->getIndex().y() * config.gridResolution,
                                   u->getHeight());
        
        // Visit each edge exiting u
        for(TraversabilityNodeBase *v : u->getConnections())
        {   
            //skip all non traversable nodes. They will retain the maximum cost.
            if(v->getType() != TraversabilityNodeBase::TRAVERSABLE)
                continue;
            
            const Eigen::Vector3d vPos(v->getIndex().x() * config.gridResolution,
                                       v->getIndex().y() * config.gridResolution,
                                       v->getHeight());

            const double distance = getHeuristicDistance(vPos, uPos, config);
            double distance_through_u = dist + distance;
            
            if (outDistances.find(v) == outDistances.end() || 
                distance_through_u < outDistances[v])
            {
                vertexQ.erase(std::make_pair(outDistances[v], v));
                outDistances[v] = distance_through_u;
                vertexQ.insert(std::make_pair(outDistances[v], v));
            }
        }
    }
}

double Dijkstra::getHeuristicDistance(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                            const TraversabilityConfig& config)
{
    return (a - b).norm();
}

    
}
