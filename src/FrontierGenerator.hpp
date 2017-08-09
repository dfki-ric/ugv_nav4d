#pragma once
#include "Planner.hpp"
#include <base/samples/RigidBodyState.hpp>
#include <maps/grid/TraversabilityMap3d.hpp>
#include "TravGenNode.hpp"
#include "Config.hpp"


namespace ugv_nav4d 
{
struct NodeWithOrientation;
struct NodeWithOrientationAndCost;
class EnvironmentXYZTheta;

class FrontierGenerator
{
    
public:
    FrontierGenerator(const TraversabilityConfig& travConf,
                      const CostFunctionParameters& costParams);
    
    template <maps::grid::MLSConfig::update_model SurfacePatch>
    void updateMap(const maps::grid::MLSMap<SurfacePatch>& mls)
    {
        mlsMap.reset(new TraversabilityGenerator3d::MLGrid(mls));
        travGen.setMLSGrid(mlsMap);
    }
    
    /** @param robotPos in mls coordinates */
    void updateRobotPos(const base::Vector3d& robotPos);
    
    /** @param goalPos in mls coordinates */
    void updateGoalPos(const base::Vector3d& goalPos);
    
    void updateCostParameters(const CostFunctionParameters& params);
    
    /** Adds traversable patches to the mls below the robot.
     * @param body2Mls Location of the body in mls coordinates
     * @param patchRadius Radius of a circle around the robot which will be filled with traversable patches*/
    void setInitialPatch(const Eigen::Affine3d &body2Mls, double patchRadius);
    
    /** Calculate a list of all frontiers that can be visited. Sorted by calculateCost() */
    std::vector<base::samples::RigidBodyState> getNextFrontiers();
    
    //just for debugging
    maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase* > getTraversabilityBaseMap() const;
    
    const TraversabilityConfig& getConfig() const;
    
private:

    /** find all frontier nodes*/
    std::vector<const TravGenNode*> getFrontierPatches() const;

    /** Get all traversable patches next to the frontier patches*/
    std::vector<NodeWithOrientation> getCandidatesFromFrontierPatches(const std::vector<NodeWithOrientation> &frontiers) const;
    
    /**Figure out goal orientation for each node in @p frontier */
    std::vector<NodeWithOrientation> getFrontierOrientation(const std::vector<const TravGenNode*>& frontier) const;
    
    /**Figure out a node that we can stand on without collisions for each node in @p nodes. */
    std::vector<NodeWithOrientation> getCollisionFreeNeighbor(const std::vector<NodeWithOrientation>& nodes) const;
    
    std::vector<NodeWithOrientation> getNodesWithoutCollision(const std::vector<NodeWithOrientation>& nodes) const;
    
    std::vector<NodeWithOrientation> removeDuplicates(const std::vector<NodeWithOrientation>& nodes) const;
    
    /** TODO describe what cost contains and what is a good/bad value */
    std::vector<NodeWithOrientationAndCost> calculateCost(const TravGenNode* startNode,
                                                          const base::Vector3d& goalPos,
                                                          const std::vector<NodeWithOrientation>& nodes) const;
    
    /**Sort nodes according to node.cost */
    std::vector<NodeWithOrientationAndCost> sortNodes(const std::vector<NodeWithOrientationAndCost>& nodes) const;
    
    /**convert to rbs */
    std::vector<base::samples::RigidBodyState> getPositions(const std::vector<NodeWithOrientationAndCost>& nodes) const;
    
    /** Calculate the number of patches that can still be explored in the vicinity
     * of @p node.
     * The number is normalized to [0..1]
     * 1 means fully explored, 0 means not explored at all*/
    double calcExplorablePatches(const TravGenNode* node) const;

    
    /**Distance between @p node and point @p p */
    double distToPoint(const TravGenNode* node, const base::Vector3d& p) const;
    
    base::Vector3d nodeCenterPos(const TravGenNode* node) const;

private:
    CostFunctionParameters costParams;
    TraversabilityConfig travConf;
    TraversabilityGenerator3d travGen;
    boost::shared_ptr<TraversabilityGenerator3d::MLGrid> mlsMap;
    base::Vector3d robotPos;
    base::Vector3d goalPos;//FIXME bad name, this is the point somewhere in the distance that indicates the direction of exploration
    
    /**When the robot cannot stand on a frontier patch (e.g. due to the obstacle check failing)
     * we explore the neighborhood of the patch to find a patch that we can stand on. 
     * This variable is the radius of the neighborhood that is explored.
     * @note this parameter heavily impacts performance (it determines the size of an innermost loop)*/
    const double maxNeighborDistance = 0.3;
};


}