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
    typedef maps::grid::MLSMapBase MLGrid;
    
protected:
    
    typedef MLGrid::CellType Cell;
    typedef MLGrid::View View;
    typedef View::CellType ViewCell;
    typedef MLGrid::PatchType Patch;
    
    
    std::shared_ptr<MLGrid > mlsGrid;
    bool addInitialPatch;
    Eigen::Affine3d initialPatch2Mls;
    double patchRadius;
    
    maps::grid::TraversabilityMap3d<TravGenNode*> trMap;
    int currentNodeId = 0; //used while expanding
    
    std::vector<TravGenNode *> growList;
    
    bool computePlaneRansac(TravGenNode &node);
    double computeSlope(const Eigen::Hyperplane< double, int(3) >& plane) const;
    Eigen::Vector3d computeSlopeDirection(const Eigen::Hyperplane< double, int(3) >& plane) const;
    
    bool checkForObstacles(TravGenNode* node);
    
    /** @return false if no allowed orientation was found (e.g. due to extreme slope)*/
    bool computeAllowedOrientations(TravGenNode* node);
    
    static bool checkForFrontier(const TravGenNode* node);
    
    void addConnectedPatches(TravGenNode* node);

    bool getConnectedPatch(const maps::grid::Index& idx, double height, const Patch*& patch);
    
    static double interpolate(double x, double x0, double y0, double x1, double y1);
    
    TravGenNode *createTraversabilityPatchAt(maps::grid::Index idx, const double curHeight);

    void growNodes();
    
    TraversabilityConfig config;
    
    void addInitialPatchToMLS();
    
    int intersections();
    
public:
    TraversabilityGenerator3d(const TraversabilityConfig &config);

    virtual ~TraversabilityGenerator3d();

    void clearTrMap();
    
    void setInitialPatch(const Eigen::Affine3d &ground2Mls, double patchRadius);
    
    TravGenNode *generateStartNode(const Eigen::Vector3d &startPos);
    TravGenNode *findMatchingTraversabilityPatchAt(maps::grid::Index idx, const double curHeight) const;
    
    void expandAll(const Eigen::Vector3d &startPos);

    void expandAll(TravGenNode *startNode);

    virtual bool expandNode(TravGenNode *node);
    
    void setMLSGrid(std::shared_ptr<MLGrid> &grid);
    
    /**Returns the number of nodes after expansion*/
    int getNumNodes() const;
    
    const maps::grid::TraversabilityMap3d<TravGenNode *> &getTraversabilityMap() const;

    maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase* > getTraversabilityBaseMap() const;
        
    void setConfig(const TraversabilityConfig &config);


};

}
