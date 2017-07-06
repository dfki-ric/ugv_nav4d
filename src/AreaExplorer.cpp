#include "AreaExplorer.hpp"
#include "FrontierGenerator.hpp"
#include <vizkit3d_debug_drawings/DebugDrawing.h>

namespace ugv_nav4d 
{
 
AreaExplorer::AreaExplorer(std::shared_ptr< ugv_nav4d::FrontierGenerator > frontGen) :
    frontGen(frontGen)
{}

bool AreaExplorer::getFrontiers(const Eigen::Vector3d& currentRobotPosition,
                                const OrientedBox& areaToExplore,
                                std::vector<base::samples::RigidBodyState>& outFrontiers)
{

     COMPLEX_DRAWING(
        
        base::Vector3d size;
        size.x() = std::abs(areaToExplore.getBox().max().x() - areaToExplore.getBox().min().x());
        size.y() = std::abs(areaToExplore.getBox().max().y() - areaToExplore.getBox().min().y());
        size.z() = std::abs(areaToExplore.getBox().max().z() - areaToExplore.getBox().min().z());
        CLEAR_DRAWING("Exploration_Area");
        DRAW_WIREFRAME_BOX("Exploration_Area", areaToExplore.getCenter(), areaToExplore.getOrientation(), size,vizkit3dDebugDrawings::Color::amber);
     );
    std::cout << "AREA CENTER: " << areaToExplore.getCenter().transpose() << std::endl;
    
    
    frontGen->updateRobotPos(currentRobotPosition);
    frontGen->updateGoalPos(areaToExplore.getCenter());
    std::cout << "generating frontiers" << std::endl;
    outFrontiers = frontGen->getNextFrontiers();
    std::cout << "generated frontiers, count: " << outFrontiers.size() << std::endl;

    for(const base::samples::RigidBodyState& frontier : outFrontiers)
    {
        if(areaToExplore.isInside(frontier.position))
        {
            return true;
        }
    }
    return false;
}




    
}