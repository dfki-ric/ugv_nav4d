#pragma once
#include <base/Eigen.hpp>

namespace ugv_nav4d 
{
    
class Box
{
public:
    Box(const base::Vector3d& center, double size, const base::Quaterniond& orientation);
    Box(const base::Vector3d& center, const base::Vector3d& dimensions, const base::Quaterniond& orientation);
    
    bool isInside(base::Vector3d point) const;
    
    base::Vector3d center;
    base::Vector3d min;
    base::Vector3d max;
    base::Quaterniond orientation;
    
};
}