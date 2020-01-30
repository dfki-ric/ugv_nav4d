#include "AreaExplorer.hpp"
#include "FrontierGenerator.hpp"
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>

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
    
    V3DD::COMPLEX_DRAWING([&]()
    {
        base::Vector3d size;
        size.x() = std::abs(areaToExplore.getBoxWithoutOrientation().max().x() - areaToExplore.getBoxWithoutOrientation().min().x());
        size.y() = std::abs(areaToExplore.getBoxWithoutOrientation().max().y() - areaToExplore.getBoxWithoutOrientation().min().y());
        size.z() = std::abs(areaToExplore.getBoxWithoutOrientation().max().z() - areaToExplore.getBoxWithoutOrientation().min().z());
        V3DD::CLEAR_DRAWING("area_explorer_Exploration_Area");
        V3DD::DRAW_WIREFRAME_BOX("area_explorer_Exploration_Area", areaToExplore.getCenter(),
                                 areaToExplore.getOrientation(), size, V3DD::Color::amber);
    });
    
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
        std::cout << "Patches in box\n";
        for(const base::samples::RigidBodyState& frontier : frontiers)
        {
            if(areaToExplore.isInside(frontier.position))
            {
                std::cout << "Still some frontiers in box, continue exploration\n";
                return false;
            }
        }
        std::cout << "No frontiers remaining in box, stop exploration\n";
        return true;
    }
    if(frontiers.empty())
    {
        std::cout << "No patches in box and frontiers empty. Stop exploration\n";
    }
    else
    {
        std::cout << "No patches in box but some frontiers remaining. Continue exploration\n";
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
