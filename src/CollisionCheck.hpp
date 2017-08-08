#pragma once
#include "TraversabilityGenerator3d.hpp"
#include "TravGenNode.hpp"
namespace ugv_nav4d 
{

struct CollisionCheck
{
    /** @return True if no collision was found */
    static bool checkCollision(const TravGenNode* node, double zRot,
                               boost::shared_ptr<TraversabilityGenerator3d::MLGrid> mls,
                               const base::Vector3d& robotHalfSize,
                               const TraversabilityGenerator3d& travGen);
};

}