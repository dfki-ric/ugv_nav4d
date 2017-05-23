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
                                                                   bool& visitChildren, bool& abort)> f)
    {
        
        std::unordered_set<const TravGenNode*> visited;
        std::deque<const TravGenNode*> toVisit;
        
        toVisit.push_back(start);
        
        while(toVisit.size() > 0)
        {
            const TravGenNode* currentNode = toVisit.front();
            toVisit.pop_front();
            
            bool visitChildren = true;
            bool abort = false;
            f(currentNode, visitChildren, abort);
            
            if(abort)
                return;
            
            visited.insert(currentNode);
            
            if(visitChildren)
            {
                for(const maps::grid::TraversabilityNodeBase* node : currentNode->getConnections())
                {
                    const TravGenNode* travNode = static_cast<const TravGenNode*>(node);
                    assert(travNode != nullptr);
                    if(visited.find(travNode) != visited.end())
                        continue;
                    toVisit.push_back(travNode);
                }
            }
        }
    }
};
    
    
}