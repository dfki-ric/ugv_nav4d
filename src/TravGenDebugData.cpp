#include "TravGenDebugData.hpp"
#include "TraversabilityGenerator3d.hpp"
#include <iostream>
namespace ugv_nav4d_debug
{

void TravGenDebugData::setTravGen(ugv_nav4d::TraversabilityGenerator3d* gen)
{
    travGen = gen;
}
    
void TravGenDebugData::planeComputed(const ugv_nav4d::TravGenNode& node)
{
    Eigen::Vector3d pos(node.getIndex().x() * travConf.gridResolution, node.getIndex().y() * travConf.gridResolution, node.getHeight());

    if(travGen == nullptr)
    {
        std::cout << "TravGenDebugData: travGen not set. Cannot do anything!!" << std::endl;    
        return;
    }    
    pos = travGen->getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
    slopes.push_back(Eigen::Vector4d(pos.x(), pos.y(), pos.z(), node.getUserData().slope));
    Eigen::Matrix<double, 2, 3> slopeDir;
    const auto mapRot = travGen->getTraversabilityMap().getLocalFrame().rotation();
    slopeDir.row(0) << pos.x() + travConf.gridResolution/2, pos.y() + travConf.gridResolution/2, pos.z();
    slopeDir.row(1) = mapRot * node.getUserData().slopeDirection;
    slopeDirections.push_back(slopeDir);
}

}