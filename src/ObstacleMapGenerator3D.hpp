#pragma once
#include "TraversabilityGenerator3d.hpp"

namespace ugv_nav4d
{
    class ObstacleMapGenerator3D : public TraversabilityGenerator3d
    {
    public:
        ObstacleMapGenerator3D(const TraversabilityConfig &config);
        virtual ~ObstacleMapGenerator3D();
        virtual bool expandNode(TravGenNode *node) override;
    };
}