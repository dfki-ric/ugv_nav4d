#include "CollisionCheck.hpp"
#include <base/Eigen.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.h>
namespace ugv_nav4d 
{
bool CollisionCheck::checkCollision(const TravGenNode* node, double zRot,
                                    boost::shared_ptr< TraversabilityGenerator3d::MLGrid > mls,
                                    const base::Vector3d& robotHalfSize,
                                    const TraversabilityGenerator3d& travGen)
{
    // calculate robot position in local grid coordinates
    // TODO make this a member method of GridMap
    maps::grid::Vector3d robotPosition;
    robotPosition <<
            (node->getIndex().cast<double>() + Eigen::Vector2d(0.5, 0.5)).cwiseProduct(travGen.getTraversabilityMap().getResolution()),
            node->getHeight() +  robotHalfSize.z();
    
    const Eigen::Vector3d planeNormal = node->getUserData().plane.normal();
    assert(planeNormal.allFinite()); 

    //FIXME names
    const Eigen::Quaterniond zRotAA( Eigen::AngleAxisd(zRot, Eigen::Vector3d::UnitZ()) ); // TODO these could be precalculated
    const Eigen::Quaterniond rotAA = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), planeNormal);
    const Eigen::Quaterniond rotQ = rotAA * zRotAA;

    // further calculations are more efficient with rotation matrix:
    const Eigen::Matrix3d rot = rotQ.toRotationMatrix();
    
    //find min/max for bounding box
    Eigen::Vector3d extends(robotHalfSize);
    extends.z() /= 2.0; //FIXME z is divided by 2 to avoid intersecting the floor, there has to be a better way?
    extends= rot.cwiseAbs() * extends;
    const Eigen::Vector3d min = robotPosition - extends;
    const Eigen::Vector3d max = robotPosition + extends;
    
    const Eigen::AlignedBox3d aabb(min, max); //aabb around the rotated robot bounding box    

    
    const Eigen::Matrix3d rotInv = rot.transpose();
    bool intersects = false;
    mls->intersectAABB_callback(aabb,
        [&rotInv, &intersects, &robotPosition, &rotQ, &rot, mls, &robotHalfSize]
        (const maps::grid::Index& idx, const maps::grid::SurfacePatchBase& p)
        {
            //FIXME this actually only tests if the top of the patch intersects with the robot
            maps::grid::Vector3d pos;
            double z = p.getMax();
            pos << (idx.cast<double>() + Eigen::Vector2d(0.5, 0.5)).cwiseProduct(mls->getResolution()), z;
            //transform pos into coordinate system of oriented bounding box
            pos -= robotPosition;
            pos = rotInv * pos;
            
            if((abs(pos.array()) <= robotHalfSize.array()).all())
            {
                //found at least one patch that is inside the oriented boundingbox
                intersects = true;
                return true;//abort intersection check
            }
            return false; //continue intersection check
        });
    
    return !intersects;
}

}