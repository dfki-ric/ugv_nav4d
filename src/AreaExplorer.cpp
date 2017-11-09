#include "AreaExplorer.hpp"
#include "FrontierGenerator.hpp"
#include <vizkit3d_debug_drawings/DebugDrawing.h>

namespace ugv_nav4d 
{
 
AreaExplorer::AreaExplorer(std::shared_ptr< ugv_nav4d::FrontierGenerator > frontGen) :
    frontGen(frontGen)
{}

void AreaExplorer::setInitialPatch(const Eigen::Affine3d& body2Mls, double patchRadius)
{
    Eigen::Affine3d ground2Body(Eigen::Affine3d::Identity());
    ground2Body.translation() = Eigen::Vector3d(0, 0, -frontGen->getConfig().distToGround);
    
    frontGen->setInitialPatch(body2Mls * ground2Body , patchRadius);
}

bool AreaExplorer::getFrontiers(const Eigen::Vector3d& body2Mls,
                                const OrientedBox& areaToExplore,
                                std::vector<base::samples::RigidBodyState>& outFrontiers)
{   
    Eigen::Vector3d ground2Mls(body2Mls);
    ground2Mls.z() -= frontGen->getConfig().distToGround;
    
     COMPLEX_DRAWING(
        
        base::Vector3d size;
        size.x() = std::abs(areaToExplore.getBox().max().x() - areaToExplore.getBox().min().x());
        size.y() = std::abs(areaToExplore.getBox().max().y() - areaToExplore.getBox().min().y());
        size.z() = std::abs(areaToExplore.getBox().max().z() - areaToExplore.getBox().min().z());
        CLEAR_DRAWING("Exploration_Area");
        DRAW_WIREFRAME_BOX("Exploration_Area", areaToExplore.getCenter(), areaToExplore.getOrientation(), size,vizkit3dDebugDrawings::Color::amber);
     );
    
    oldStarts.push_back(ground2Mls);
    generateFrontiers(oldStarts, areaToExplore, outFrontiers);
    
    //move to robot height
    for(base::samples::RigidBodyState& frontier : outFrontiers)
    {
        frontier.position.z() += frontGen->getConfig().distToGround;
    }
    
    std::cout << "generated frontiers, count: " << outFrontiers.size() << std::endl;

    
    return !isAreaExplored(areaToExplore, outFrontiers);

}


bool AreaExplorer::isAreaExplored(const OrientedBox& areaToExplore, const std::vector<base::samples::RigidBodyState>& frontiers) const
{
    if(frontGen->patchesInBox(areaToExplore))
    {
        for(const base::samples::RigidBodyState& frontier : frontiers)
        {
            if(areaToExplore.isInside(frontier.position))
            {
                return false;
            }
        }
        return true;
    }
    
    return frontiers.empty();
}

    
void AreaExplorer::generateFrontiers(std::vector< Eigen::Vector3d > starts,
                                     const OrientedBox& areaToExplore,
                                     std::vector< base::samples::RigidBodyState >& outFrontiers)
{
    std::cout << "genarting frontier iterations" << std::endl;
    for(int i = starts.size() - 1; i >= 0; --i)
    {
        std::cout << "iteration: " << i << std::endl;
        const base::Vector3d start = starts[i];
        frontGen->updateRobotPos(start);
        frontGen->updateGoalPos(areaToExplore.getCenter());
        std::cout << "generating frontiers" << std::endl;
        outFrontiers = frontGen->getNextFrontiers();
        
        if(outFrontiers.size() > 0)
        {
            std::cout << "got frontiers at iteration: " << i << std::endl;
            return;
        }
    }
}

    
    
}
