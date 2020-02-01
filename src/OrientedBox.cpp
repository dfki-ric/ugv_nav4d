#include "OrientedBox.hpp"

namespace ugv_nav4d {

OrientedBox::OrientedBox(const base::Vector3d& center, const base::Vector3d& dimensions, const base::Quaterniond& orientation) : 
    center(center), orientation(orientation)
{
    halfSize = dimensions / 2.0;
    Eigen::Vector3d min = -halfSize + center;
    Eigen::Vector3d max = halfSize + center;
    
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


bool OrientedBox::isInside(base::Vector3d p, double scaling) const
{
    //move p to box coorindate system and rotate it
    p = p - center;
    p = orientation.inverse() * p;
    const Eigen::AlignedBox3d localBox(-(halfSize * scaling), (halfSize * scaling));
    return localBox.contains(p);
}


}
