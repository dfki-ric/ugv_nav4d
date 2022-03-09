#pragma once

#include <maps/grid/MLSMap.hpp>
#include <boost/shared_ptr.hpp>
#include "TraversabilityConfig.hpp"
#include "TravGenNode.hpp"

namespace ugv_nav4d
{

class TraversabilityGenerator3d
{
public:
    // TODO use MLSMapPrecalculated and actually use slope information?
//    typedef maps::grid::MultiLevelGridMap< maps::grid::SurfacePatchBase > MLGrid;
    typedef maps::grid::MLSMapSloped MLGrid;
    
protected:
    
    typedef MLGrid::CellType Cell;
    typedef MLGrid::View View;
    typedef View::CellType ViewCell;
    typedef MLGrid::PatchType Patch;
    
    
    std::shared_ptr<MLGrid > mlsGrid;
    bool addInitialPatch;
    Eigen::Affine3d initialPatch2Mls;
    double patchRadius;

    std::vector<TravGenNode*> obstacleNodesGrowList;
    
    maps::grid::TraversabilityMap3d<TravGenNode*> trMap;
    int currentNodeId = 0; //used while expanding
    
    std::vector<TravGenNode *> frontierNodesGrowList;
    
    bool computePlaneRansac(TravGenNode &node);
    double computeSlope(const Eigen::Hyperplane< double, int(3) >& plane) const;
    Eigen::Vector3d computeSlopeDirection(const Eigen::Hyperplane< double, int(3) >& plane) const;
    
    bool checkStepHeight(TravGenNode* node);
    
    /** @return false if no allowed orientation was found (e.g. due to extreme slope)*/
    bool computeAllowedOrientations(TravGenNode* node);
    
    static bool checkForFrontier(const TravGenNode* node);
    
    void addConnectedPatches(TravGenNode* node);

    bool getConnectedPatch(const maps::grid::Index& idx, double height, const Patch*& patch);
    
    static double interpolate(double x, double x0, double y0, double x1, double y1);
    
    TravGenNode *createTraversabilityPatchAt(maps::grid::Index idx, const double curHeight);

    void growNodes();

    void inflateObstacles();
    
    TraversabilityConfig config;
    
    void addInitialPatchToMLS();
    
    int intersections();
    
public:
    TraversabilityGenerator3d(const TraversabilityConfig &config);

    virtual ~TraversabilityGenerator3d();

    void clearTrMap();
    
    void setInitialPatch(const Eigen::Affine3d &ground2Mls, double patchRadius);
    
    virtual TravGenNode *generateStartNode(const Eigen::Vector3d &startPos);
    TravGenNode *findMatchingTraversabilityPatchAt(maps::grid::Index idx, const double curHeight) const;
    
    
    /**Expand the map starting from all given @p positions */
    void expandAll(const std::vector<Eigen::Vector3d>& positions);
    
    void expandAll(const Eigen::Vector3d &startPos);
    
    
    /**Expands the map starting at @p startPos.
     * Expansion will stop if a distance of @p expandDist is reached. I.e. this will expand all nodes
     * in a circle of radius @p expandDist around @p startPos.*/
    void expandAll(const Eigen::Vector3d &startPos, const double expandDist);
    
    void expandAll(TravGenNode *startNode);
    
    /** @param expandDist How far should the map be expanded? If negative the whole map will be expanded. */
    void expandAll(TravGenNode *startNode, const double expandDist);

    virtual bool expandNode(TravGenNode *node);
    
    void setMLSGrid(std::shared_ptr<MLGrid> &grid);
    
    /**Returns the number of nodes after expansion*/
    int getNumNodes() const;
    
    const maps::grid::TraversabilityMap3d<TravGenNode *> &getTraversabilityMap() const;

        
    void setConfig(const TraversabilityConfig &config);


};

}
