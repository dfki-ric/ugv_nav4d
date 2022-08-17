#pragma once
#include <traversability_generator3d/TraversabilityGenerator3d.hpp>

namespace ugv_nav4d
{
    class ObstacleMapGenerator3D : public traversability_generator3d::TraversabilityGenerator3d
    {
    public:
        ObstacleMapGenerator3D(const traversability_generator3d::TraversabilityConfig &config);
        virtual ~ObstacleMapGenerator3D();
        virtual bool expandNode(traversability_generator3d::TravGenNode *node) override;
//         virtual traversability_generator3d::TravGenNode *generateStartNode(const Eigen::Vector3d &startPos) override;
        
    private:
        
        /** @return true if obstacle check passed */
        bool obstacleCheck(const traversability_generator3d::TravGenNode* node) const;
    };
}
