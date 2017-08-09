#pragma once
#include <memory>
#include <base/samples/RigidBodyState.hpp>
#include "OrientedBox.hpp"

namespace ugv_nav4d 
{
class FrontierGenerator;
    

class AreaExplorer
{
public:
    AreaExplorer(std::shared_ptr<FrontierGenerator> frontGen);
    
    void setInitialPatch(const Eigen::Affine3d& body2Mls, double patchRadius);

        
    /** Get a list of all frontiers sorted by how good they are to explore @p area
     * @param body2Mls current position of the robot in mls coordinates
     * @param areaToExplore in mls coordinates
     * @param[out] outFrontiers A list of possible frontiers that can be explored.
     *                         The list is sorted by how "good" it would be to explore them to uncover more of @þ area.
     *                         The frontiers are in the mls frame but adjusted for the robot height.
     *                         I.e. they hover above the mls at the height of the robot
     * @return False if there are no more frontiers inside the specified area. True otherwise
     */
    bool getFrontiers(const Eigen::Vector3d& body2Mls,
                      const OrientedBox& areaToExplore,
                      std::vector<base::samples::RigidBodyState>& outFrontiers);
    
private:
    std::shared_ptr<FrontierGenerator> frontGen;

};

}