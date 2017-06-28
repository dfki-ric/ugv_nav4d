#include "OrientedBox.hpp"

namespace ugv_nav4d {

OrientedBox::OrientedBox(const base::Vector3d& center, const base::Vector3d& dimensions, const base::Quaterniond& orientation) : 
    center(center), orientation(orientation)
{
    const base::Vector3d halfSize = dimensions / 2.0;
    min = -halfSize;
    max = halfSize;
}

OrientedBox::OrientedBox(const base::Vector3d& center, double size, const base::Quaterniond& orientation) :
    center(center), orientation(orientation)
{
    const double halfSize = size / 2.0;
    min.array() = -halfSize;
    max.array() = halfSize;
}


bool OrientedBox::isInside(base::Vector3d p) const
{
    //move p to box coorindate system 
    p = p - center;
    p = orientation.inverse() * p;
    
    return p.x() >= min.x() && p.x() <= max.x() &&
           p.y() >= min.y() && p.y() <= max.y() &&
           p.z() >= min.z() && p.z() <= max.z();
}


}