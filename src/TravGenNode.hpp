#pragma once
#include <base/Eigen.hpp>
#include <maps/grid/TraversabilityMap3d.hpp>
#include <boost/serialization/serialization.hpp>

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
    
    /** Serializes the members of this class*/
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & plane.offset();
        ar & plane.normal().x();
        ar & plane.normal().y();
        ar & plane.normal().z();
        ar & slope;
        ar & slopeDirection.x();
        ar & slopeDirection.y();
        ar & slopeDirection.z();
        ar & slopeDirectionAtan2;
        ar & id;
    }
};

typedef maps::grid::TraversabilityNode<TravGenTrackingData> TravGenNode;

}
