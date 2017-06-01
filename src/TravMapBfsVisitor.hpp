#pragma once
#include "TravGenNode.hpp"
#include <functional>
#include <unordered_set>
#include <deque>

namespace ugv_nav4d
{
    
//FIXME should be moved to envire::maps ?!

/** visit travgen nodes in bfs order until some criterion is met*/
class TravMapBfsVisitor
{
public:
    
    /** @param f is called for each visited node.
     *           set @p visitChildren to false to avoid visiting the children of @p currentNode
     *           set @p abort to true to abort the bfs search*/
    static void visit(const TravGenNode* start, std::function<void(const TravGenNode* currentNode,
                                                                   bool& visitChildren, bool& abort, std::size_t distToRoot)> f)
    {
        
        std::unordered_set<const TravGenNode*> visited;
        std::deque<const TravGenNode*> toVisit;
        std::unordered_map<const TravGenNode*, std::size_t> distToRoot;
        toVisit.push_back(start);
        visited.insert(start);
        distToRoot[start] = 0;
        
        while(toVisit.size() > 0)
        {
            const TravGenNode* currentNode = toVisit.front();
            toVisit.pop_front();
            
            bool visitChildren = true;
            bool abort = false;
            f(currentNode, visitChildren, abort, distToRoot[currentNode]);
            
            if(abort)
                return;

            if(visitChildren)
            {
                for(const maps::grid::TraversabilityNodeBase* node : currentNode->getConnections())
                {
                    const TravGenNode* travNode = static_cast<const TravGenNode*>(node);
                    assert(travNode != nullptr);
                    if(visited.find(travNode) != visited.end())
                        continue;
                    visited.insert(travNode); //if we mark it as visited after the visit it might be visited multiple times
                    toVisit.push_back(travNode);
                    distToRoot[travNode] = distToRoot[currentNode] + 1;
                }
            }
        }
    }
};
    
    
}