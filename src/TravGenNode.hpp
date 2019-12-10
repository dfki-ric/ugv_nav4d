#pragma once
#include <base/Eigen.hpp>
#include <maps/grid/TraversabilityMap3d.hpp>
#include <boost/serialization/serialization.hpp>
#include <base/Angle.hpp>

namespace ugv_nav4d
{

/**Node struct for TraversabilityMap3d */
struct TravGenTrackingData
{
    /** The plane that has been fitted to the mls at the location of this node */
    Eigen::Hyperplane<double, 3> plane;
    
    /** slope of the plane */
    double slope;
    
    /** normalized direction of the slope. Only valid if slope > 0 */
    Eigen::Vector3d slopeDirection;
    
    /** The atan2(slopeDirection.y(), slopeDirection.x()), i.e. angle of slopeDirection projected on the xy plane.
     * Precomputed for performance reasons */
    double slopeDirectionAtan2; 
    
    /** continuous unique id  that can be used as index for additional metadata */
    size_t id; 
    
    /**Some orientations might be forbidden on this patch (e.g. due to slope). This vector
     * contains all orientations that are allowed */
    std::vector<base::AngleSegment> allowedOrientations;
    
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
        ar & allowedOrientations;
    }
};

typedef maps::grid::TraversabilityNode<TravGenTrackingData> TravGenNode;

}
