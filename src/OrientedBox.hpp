#pragma once
#include <base/Eigen.hpp>

namespace ugv_nav4d 
{
    
struct OrientedBoxConfig
{
    base::Vector3d center;
    base::Vector3d dimensions;
    base::Quaterniond orientation;
};

class OrientedBox
{
public:
    OrientedBox(const OrientedBoxConfig &config);
    OrientedBox(const base::Vector3d& center, double size, const base::Quaterniond& orientation);
    OrientedBox(const base::Vector3d& center, const base::Vector3d& dimensions, const base::Quaterniond& orientation);

    const Eigen::Vector3d &getCenter() const
    {
        return center;
    }
    
    const Eigen::Quaterniond& getOrientation() const
    {
        return orientation;
    }
    
    const Eigen::AlignedBox3d& getBox() const
    {
        return box;
    }

    bool isInside(base::Vector3d point) const;

private:    
    Eigen::AlignedBox3d box;
    Eigen::Vector3d center;
    Eigen::Quaterniond orientation;
};
}