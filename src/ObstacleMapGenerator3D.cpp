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


TravGenNode *ObstacleMapGenerator3D::generateStartNode(const Eigen::Vector3d &startPos) 
{
    //HACK This method only exists to ensure consistent behavior with the PoseWatchdog 
    //     for the final entern demo. This should ***NOT*** remain 
    
    //if we dont do this check here, expansion will fail silently when expandAll is called
    //downstream. In that case setStart() would not fail. If setStart does not fail,
    //the planner does not generate a rescue trajectory, even though it should!
    
    
    TravGenNode* node = TraversabilityGenerator3d::generateStartNode(startPos);
    
    //do additional obstacle check to ensure consistent behavior with PoseWatchdog
    if(node)
    {
        //if it is expanded, just do the additional obstacle check
        if(node->isExpanded() && !obstacleCheck(node))
        {
            //FIXME probably this never happens because generateStartNode should not return expanded nodes?! Maybe it happens if it returned an already existing node
            return nullptr;
        }
        else
        {
            //expansion contains the additional obstacle check that we want
            const bool expansionOk = expandNode(node);
            //but we have to return a non-expanded node, otherwise downstream expandAll() calls will fail
            node->setNotExpanded();

            if(!expansionOk)
                return nullptr;
        }
    }
    return node;
}

    
bool ObstacleMapGenerator3D::expandNode(TravGenNode *node)
{
    node->setExpanded();

    if(node->getType() == TraversabilityNodeBase::UNKNOWN
        || node->getType() == TraversabilityNodeBase::OBSTACLE)
    {
        return false;
    }
    
    if(node->getUserData().slope > config.maxSlope)
        return false;

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
