#include "OrientedBox.hpp"

namespace ugv_nav4d {

OrientedBox::OrientedBox(const base::Vector3d& center, const base::Vector3d& dimensions, const base::Quaterniond& orientation) : 
    center(center), orientation(orientation)
{
    const base::Vector3d halfSize = dimensions / 2.0;
    Eigen::Vector3d min = -halfSize;
    Eigen::Vector3d max = halfSize;
    
    box = Eigen::AlignedBox3d(min, max);
}

OrientedBox::OrientedBox(const base::Vector3d& center, double size, const base::Quaterniond& orientation) :
    OrientedBox(center, base::Vector3d(size, size, size), orientation)
{
}

OrientedBox::OrientedBox(const OrientedBoxConfig& config) :
    OrientedBox(config.center, config.dimensions, config.orientation)
{

}


bool OrientedBox::isInside(base::Vector3d p) const
{
    //move p to box coorindate system 
    p = p - center;
    p = orientation.inverse() * p;
    
    return box.contains(p);
}


}