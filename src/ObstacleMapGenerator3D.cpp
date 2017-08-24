#include "ObstacleMapGenerator3D.hpp"
#include <vizkit3d_debug_drawings/DebugDrawing.h>
#include <vizkit3d_debug_drawings/DebugDrawingColors.h>

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

    if(node->getType() == TraversabilityNodeBase::UNKNOWN
        || node->getType() == TraversabilityNodeBase::OBSTACLE)
    {
        return false;
    }

    if(!obstacleCheck(node))
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


bool ObstacleMapGenerator3D::obstacleCheck(const TravGenNode* node) const
{
    if(node->getUserData().slope > config.maxSlope)
        return false;
    
    //check if there is an mls patch above the ground
    Eigen::Vector3d nodePos;
    if(!trMap.fromGrid(node->getIndex(), nodePos, node->getHeight()))
        throw std::runtime_error("ObstacleMapGenerator3D: Internal error node out of grid");

    Eigen::Vector3d min(-config.gridResolution/2.0 + 1e-5, -config.gridResolution / 2.0 + 1e-5, config.maxStepHeight);
    Eigen::Vector3d max(config.gridResolution/2.0 - 1e-5, config.gridResolution/2.0 - 1e-5, config.robotHeight);
    
    
    min += nodePos;
    max += nodePos;
    
    const Eigen::AlignedBox3d boundingBox(min, max);
    
//     DRAW_AABB("obstacle map box", boundingBox, vizkit3dDebugDrawings::Color::cyan);
    size_t numIntersections = 0;
    const View area = mlsGrid->intersectCuboid(Eigen::AlignedBox3d(min, max), numIntersections);
    if(numIntersections > 0)
        return false;
    
    return true;
}


}