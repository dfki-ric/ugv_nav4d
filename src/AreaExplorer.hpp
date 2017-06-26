#pragma once
#include <memory>
#include <base/Eigen.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace ugv_nav4d 
{
class FrontierGenerator;
    

class Area
{
public:
    virtual Eigen::Vector3d getCenter() const = 0;
    virtual bool isInside(const Eigen::Vector3d& point) const = 0;
    virtual ~Area() {}
};

class AreaExplorer
{
public:
    AreaExplorer(std::shared_ptr<FrontierGenerator> frontGen);
    
        
    /** Get a list of all frontiers sorted by how good they are to explore @p area
     * @param[out] outFrontiers A list of possible frontiers that can be explored.
     *                         The list is sorted by how "good" it would be to explore them to uncover more of @þ area
     * @return False if there are no more frontiers inside the specified area. True otherwise
     */
    bool getFrontiers(const Eigen::Vector3d& currentRobotPosition,
                      const std::shared_ptr<Area> areaToExplore,
                      std::vector<base::samples::RigidBodyState>& outFrontiers);
    
private:
    std::shared_ptr<FrontierGenerator> frontGen;

};

}