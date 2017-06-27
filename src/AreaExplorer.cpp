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
        if(areaToExplore.isInside(frontier.position))
        {
            return true;
        }
    }
    return false;
}

Box::Box(const base::Vector3d& center, const base::Vector3d& dimensions, const base::Quaterniond& orientation) : 
    center(center), orientation(orientation)
{
    const base::Vector3d halfSize = dimensions / 2.0;
    min = -halfSize;
    max = halfSize;
}

Box::Box(const base::Vector3d& center, double size, const base::Quaterniond& orientation) :
    center(center), orientation(orientation)
{
    const double halfSize = size / 2.0;
    min.array() = -halfSize;
    max.array() = halfSize;
}


bool Box::isInside(base::Vector3d p) const
{
    //move p to box coorindate system 
    p = p - center;
    p = orientation.inverse() * p;
    
    return p.x() >= min.x() && p.x() <= max.x() &&
           p.y() >= min.y() && p.y() <= max.y() &&
           p.z() >= min.z() && p.z() <= max.z();
}



    
}