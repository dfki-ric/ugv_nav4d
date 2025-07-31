#include "Dijkstra.hpp"
#include <maps/grid/TraversabilityMap3d.hpp>
#include <traversability_generator3d/TraversabilityConfig.hpp>
#include <queue>
using namespace maps::grid;

namespace ugv_nav4d
{
void Dijkstra::computeCost(const TraversabilityNodeBase* source,
                           std::unordered_map<const TraversabilityNodeBase*, double>& outDistances,
                           const traversability_generator3d::TraversabilityConfig& config)
{
    outDistances.clear();
    outDistances[source] = 0.0;

    // Use a priority queue for efficiency
    std::priority_queue<std::pair<double, const TraversabilityNodeBase*>,
                        std::vector<std::pair<double, const TraversabilityNodeBase*>>,
                        std::greater<>> vertexQ;
    vertexQ.emplace(0.0, source);

    std::unordered_set<const TraversabilityNodeBase*> visited;

    while (!vertexQ.empty())
    {
        const double dist = vertexQ.top().first;
        const TraversabilityNodeBase* u = vertexQ.top().second;
        vertexQ.pop();

        // Skip nodes already processed
        if (visited.find(u) != visited.end())
            continue;
        visited.insert(u);

        const Eigen::Vector3d uPos(u->getIndex().x() * config.gridResolution,
                                   u->getIndex().y() * config.gridResolution,
                                   u->getHeight());

        // Visit each edge exiting u
        for (TraversabilityNodeBase* v : u->getConnections())
        {
            if (v->getType() != TraversabilityNodeBase::TRAVERSABLE)
                continue;

            const Eigen::Vector3d vPos(v->getIndex().x() * config.gridResolution,
                                       v->getIndex().y() * config.gridResolution,
                                       v->getHeight());

            const double distance = (vPos - uPos).norm();
            double distance_through_u = dist + distance;

            if (outDistances.find(v) == outDistances.end() || distance_through_u < outDistances[v])
            {
                outDistances[v] = distance_through_u;
                vertexQ.emplace(outDistances[v], v);
            }
        }
    }
}
}

