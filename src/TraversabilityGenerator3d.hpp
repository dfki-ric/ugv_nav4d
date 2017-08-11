#pragma once

#include <maps/grid/MLSMap.hpp>
#include <boost/shared_ptr.hpp>
#include "TraversabilityConfig.hpp"
#include "TravGenNode.hpp"

//#define GENERATE_DEBUG_DATA
#include "UgvDebug.hpp"
#include "TravGenDebugData.hpp"


namespace ugv_nav4d
{

class TraversabilityGenerator3d
{
public:

    UGV_DEBUG(
        ugv_nav4d_debug::TravGenDebugData debugData;
    )
    typedef maps::grid::MultiLevelGridMap< maps::grid::SurfacePatchBase > MLGrid;
    
private:
    
    typedef MLGrid::CellType Cell;
    typedef MLGrid::View View;
    typedef View::CellType ViewCell;
    typedef MLGrid::PatchType Patch;
    
    
    boost::shared_ptr<MLGrid > mlsGrid;
    bool addInitialPatch;
    Eigen::Affine3d initialPatch2Mls;
    double patchRadius;
    
    maps::grid::TraversabilityMap3d<TravGenNode*> trMap;
    int currentNodeId = 0; //used while expanding
    
    bool computePlaneRansac(TravGenNode &node);
    double computeSlope(const Eigen::Hyperplane< double, int(3) >& plane) const;
    Eigen::Vector3d computeSlopeDirection(const Eigen::Hyperplane< double, int(3) >& plane) const;
    
    bool checkForObstacles(TravGenNode* node);
    
    /** @return false if no allowed orientation was found (e.g. due to extreme slope)*/
    bool computeAllowedOrientations(TravGenNode* node);
    
    bool checkForFrontier(TravGenNode* node);
    
    void addConnectedPatches(TravGenNode* node);

    bool getConnectedPatch(const maps::grid::Index& idx, double height, const Patch*& patch);
    
    double interpolate(double x, double x0, double y0, double x1, double y1) const;
    
    TraversabilityConfig config;
    
    void addInitialPatchToMLS();
    
public:
    TraversabilityGenerator3d(const TraversabilityConfig &config);

    ~TraversabilityGenerator3d();

    void clearTrMap();
    
    void setInitialPatch(const Eigen::Affine3d &ground2Mls, double patchRadius);
    
    TravGenNode *generateStartNode(const Eigen::Vector3d &startPos);
    void expandAll(const Eigen::Vector3d &startPos);

    void expandAll(TravGenNode *startNode);

    bool expandNode(TravGenNode *node);
    
    void setMLSGrid(boost::shared_ptr<MLGrid> &grid);
    
    /**Returns the number of nodes after expansion*/
    int getNumNodes() const;
    
    const maps::grid::TraversabilityMap3d<TravGenNode *> &getTraversabilityMap() const;

    maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase* > getTraversabilityBaseMap() const;
        
    void setConfig(const TraversabilityConfig &config);
    
    /**Compute heuristic cost from @p source to all reachable nodes.
     * @param outDistances mapping from node id to cost*/
    void dijkstraComputeCost(const TravGenNode* source, std::vector<double> &outDistances, const double maxDist) const;

    /**Compute heuristic cost from @p source to all reachable nodes.
     * @param outDistances mapping from node id to cost
     * This version skips any node, that is only reachable over
     * a frontier patch
     */
    void dijkstraComputeCostNoDoubleFrontierts(const TravGenNode* source, std::vector<double> &outDistances, const double maxDist) const;
    
    /** Returns distance from @p a to @p b based on config.heuristicType */
    double getHeuristicDistance(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const;
    
protected:
    int intersections();
};

}
