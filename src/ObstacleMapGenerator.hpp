#pragma once
#include "BaseMapGenerator.hpp"
#include "ObstacleNode.hpp"

namespace ugv_nav4d
{
class ObstacleMapGenerator : public BaseMapGenerator<ObstacleNode>
{
public:
    
    virtual bool expandNode(ObstacleNode* node) override
    {
    }
        
    virtual ObstacleNode* generateStartNode(const Eigen::Vector3d& startPos) override
    {
    }
};
}