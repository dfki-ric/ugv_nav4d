#pragma once
#include <base/Eigen.hpp>
#include <maps/grid/TraversabilityMap3d.hpp>

namespace ugv_nav4d
{

/**Node struct for TraversabilityMap3d */
struct TravGenTrackingData
{
    Eigen::Hyperplane<double, 3> plane;
    double slope;
    Eigen::Vector3d slopeDirection; //normalized direction of the maximum slope. Only valid if slope > 0
    double slopeDirectionAtan2; // = atan2(slopeDirection.y(), slopeDirection.x()), i.e. angle of slopeDirection projected on the xy plane.
    size_t id; //continuous unique id  that can be used as index for additional metadata
};

typedef maps::grid::TraversabilityNode<TravGenTrackingData> TravGenNode;

}
