#include "AreaExplorer.hpp"
#include "FrontierGenerator.hpp"

namespace ugv_nav4d 
{
 
AreaExplorer::AreaExplorer(std::shared_ptr< ugv_nav4d::FrontierGenerator > frontGen) :
    frontGen(frontGen)
{}

bool AreaExplorer::getFrontiers(const Eigen::Vector3d& currentRobotPosition,
                                const OrientedBox& areaToExplore,
                                std::vector<base::samples::RigidBodyState>& outFrontiers)
{
    frontGen->updateRobotPos(currentRobotPosition);
    frontGen->updateGoalPos(areaToExplore.getCenter());
    outFrontiers = frontGen->getNextFrontiers();

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