#pragma once
#include <base/Eigen.hpp>
#include <maps/grid/TraversabilityMap3d.hpp>
#include <boost/serialization/serialization.hpp>
#include <base/Angle.hpp>

namespace ugv_nav4d
{

/**Node struct for TraversabilityMap3d */
struct ObstacleTrackingData
{
    /** Serializes the members of this class*/
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
    }
};

typedef maps::grid::TraversabilityNode<ObstacleTrackingData> ObstacleNode;

}
