#include "AreaExplorer.hpp"
#include "FrontierGenerator.hpp"

namespace ugv_nav4d 
{
 
AreaExplorer::AreaExplorer(std::shared_ptr< ugv_nav4d::FrontierGenerator > frontGen) :
    frontGen(frontGen)
{}

bool AreaExplorer::getFrontiers(const Eigen::Vector3d& currentRobotPosition,
                                const Box& areaToExplore,
                                std::vector<base::samples::RigidBodyState>& outFrontiers)
{
    frontGen->updateRobotPos(currentRobotPosition);
    frontGen->updateGoalPos(areaToExplore.center);
    outFrontiers = frontGen->getNextFrontiers();

    for(const base::samples::RigidBodyState& frontier : outFrontiers)
    {
        if(!areaToExplore.isInside(frontier.position))
        {
            return false;
        }
    }
    return true;
}

Box::Box(const base::Vector3d& center, const base::Vector3d& dimensions)
{
    const base::Vector3d halfSize = dimensions / 2.0;
    min = center - halfSize;
    max = center + halfSize;
}

Box::Box(const base::Vector3d& center, double size) : center(center)
{
    const double halfSize = size / 2.0;
    min = center.array() - halfSize;
    max = center.array() + halfSize;
}



bool Box::isInside(const base::Vector3d& p) const
{
    return p.x() >= min.x() && p.x() <= max.x() &&
           p.y() >= min.y() && p.y() <= max.y() &&
           p.z() >= min.z() && p.z() <= max.z();
}



    
}