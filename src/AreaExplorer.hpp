#pragma once
#include <memory>
#include <base/samples/RigidBodyState.hpp>
#include "Box.hpp"

namespace ugv_nav4d 
{
class FrontierGenerator;
    

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
                      const Box& areaToExplore,
                      std::vector<base::samples::RigidBodyState>& outFrontiers);
    
private:
    std::shared_ptr<FrontierGenerator> frontGen;

};

}