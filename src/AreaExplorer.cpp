#include "AreaExplorer.hpp"
#include "FrontierGenerator.hpp"

namespace ugv_nav4d 
{
 
AreaExplorer::AreaExplorer(std::shared_ptr< ugv_nav4d::FrontierGenerator > frontGen) :
    frontGen(frontGen)
{}

bool AreaExplorer::getFrontiers(const Eigen::Vector3d& currentRobotPosition,
                                const std::shared_ptr< ugv_nav4d::Area > areaToExplore,
                                std::vector<base::samples::RigidBodyState>& outFrontiers)
{
//     frontGen->updateRobotPos(currentRobotPosition);
//     frontGen->updateGoalPos(areaToExplore->getCenter());
//     frontGen->getNextFrontiers();
    
    return true;
    
}


    
}