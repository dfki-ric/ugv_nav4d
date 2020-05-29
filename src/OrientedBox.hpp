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

    const Eigen::Vector3d &getHalfSize() const 
    {
        return halfSize;
    }
    
    const Eigen::Quaterniond& getOrientation() const
    {
        return orientation;
    }
    
    /** @return the box without orientation */
    const Eigen::AlignedBox3d& getBoxWithoutOrientation() const
    {
        return box;
    }

    /** @p point in same coordinate system as center */
    bool isInside(base::Vector3d point, double scaling) const;

private:    
    Eigen::Vector3d halfSize; /**< (size of box) / 2.0 */
    Eigen::AlignedBox3d box; /**< Box without orientation*/
    Eigen::Vector3d center; /** < Center position of the box */
    Eigen::Quaterniond orientation; /** Rotation around center point */
};
}
