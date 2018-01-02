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
struct MovedNode;
class EnvironmentXYZTheta;
class OrientedBox;

class FrontierGenerator
{
    
public:
    FrontierGenerator(const TraversabilityConfig& travConf,
                      const CostFunctionParameters& costParams);
    
    template <maps::grid::MLSConfig::update_model SurfacePatch>
    void updateMap(const maps::grid::MLSMap<SurfacePatch>& mls, const maps::grid::MLSMapBase* coverage_map = nullptr)
    {
        mlsMap.reset(new TraversabilityGenerator3d::MLGrid(mls));
        travGen.setMLSGrid(mlsMap);
        coverageMap = coverage_map;
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
    const maps::grid::TraversabilityMap3d< TravGenNode *> &getTraversabilityMap() const;
    
    const TraversabilityConfig& getConfig() const;
    
    /** Returns true if there are any travMap patches inside @p box */
    bool patchesInBox(const OrientedBox& box) const;
    
private:

    /** find all frontier nodes*/
    std::vector<const TravGenNode*> getFrontierPatches() const;

    /** Get all traversable patches next to the frontier patches*/
    std::vector<NodeWithOrientation> getCandidatesFromFrontierPatches(const std::vector<NodeWithOrientation> &frontiers) const;
    
    /**Figure out goal orientation for each node in @p frontier */
    std::vector<NodeWithOrientation> getFrontierOrientation(const std::vector<const TravGenNode*>& frontier) const;
    
    /**Figure out a node that we can stand on without collisions for each node in @p nodes. */
    std::vector<MovedNode> getCollisionFreeNeighbor(const std::vector<NodeWithOrientation>& nodes) const;
        
    std::vector<MovedNode> removeDuplicates(const std::vector<MovedNode>& nodes) const;
    
    /** TODO describe what cost contains and what is a good/bad value */
    std::vector<NodeWithOrientationAndCost> calculateCost(const TravGenNode* startNode,
                                                          const base::Vector3d& goalPos,
                                                          const std::vector<MovedNode>& nodes) const;
    
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
    std::shared_ptr<TraversabilityGenerator3d::MLGrid> mlsMap;
    const maps::grid::MLSMapBase* coverageMap;
    base::Vector3d robotPos;
    base::Vector3d goalPos;//FIXME bad name, this is the point somewhere in the distance that indicates the direction of exploration
};


}
