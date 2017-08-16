#pragma once
#include "BaseMapGenerator.hpp"
#include "TravGenNode.hpp"

namespace ugv_nav4d
{
class TraversabilityMapGenerator : public BaseMapGenerator<TravGenNode>
{
private:
    
    int currentNodeId = 0; //used while expanding
    
public:
    
    virtual bool expandNode(TravGenNode* node) override;
        
    virtual TravGenNode* generateStartNode(const Eigen::Vector3d& startPos) override;
};
}