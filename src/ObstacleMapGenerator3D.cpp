#include "ObstacleMapGenerator3D.hpp"

using namespace maps::grid;

namespace ugv_nav4d
{
    
ObstacleMapGenerator3D::ObstacleMapGenerator3D(const TraversabilityConfig& config): TraversabilityGenerator3d(config)
{

}

ObstacleMapGenerator3D::~ObstacleMapGenerator3D()
{

}


    
bool ObstacleMapGenerator3D::expandNode(TravGenNode *node)
{
    node->setExpanded();

    
//         if(!checkForObstacles(node))
//         {
//             node->setType(TraversabilityNodeBase::OBSTACLE);
//             return false;
//         }
    
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
}