#include "TraversabilityMapGenerator.hpp"

using namespace maps::grid;

namespace ugv_nav4d
{
    bool TraversabilityMapGenerator::expandNode(TravGenNode* node)
    {
        node->setExpanded();
        
        if(!checkForObstacles(node))
        {
            node->setType(TraversabilityNodeBase::OBSTACLE);
            return false;
        }
        
        if(!computeAllowedOrientations(node))
        {
            node->setType(TraversabilityNodeBase::OBSTACLE);
            return false;
        }

        //add sourounding 
        addConnectedPatches(node);

        if(checkForFrontier(node))
        {
            node->setType(TraversabilityNodeBase::FRONTIER);
            return false;
        }

        node->setType(TraversabilityNodeBase::TRAVERSABLE);
        
        return true;
    }
    
    TravGenNode* TraversabilityMapGenerator::generateStartNode(const Eigen::Vector3d& startPos)
    {
        maps::grid::Index idx;
        if(!trMap.toGrid(startPos, idx))
        {
            std::cout << "TraversabilityGenerator3d::generateStartNode: Start position outside of map !" << std::endl;
            return nullptr;
        }

        //check if not already exists...
        TravGenNode *startNode = findMatchingTraversabilityPatchAt(idx, startPos.z());
        if(startNode)
        {
            std::cout << "TraversabilityGenerator3d::generateStartNode: Using existing node " << std::endl;
            return startNode;
        }

        startNode = createTraversabilityPatchAt(idx, startPos.z());
        if(!startNode)
        {
            std::cout << "TraversabilityGenerator3d::generateStartNode: Could not create travNode for given start position, no matching / not enough MSL patches" << std::endl;
            return startNode;
        }

        return startNode;
    }
    
}