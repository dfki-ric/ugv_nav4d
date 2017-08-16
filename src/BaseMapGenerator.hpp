#pragma once

#include <maps/grid/MLSMap.hpp>
#include <boost/shared_ptr.hpp>
#include <maps/grid/TraversabilityMap3d.hpp>
#include "TraversabilityConfig.hpp"



namespace ugv_nav4d
{


template <class NODE_TYPE>
class BaseMapGenerator
{
public:
    typedef maps::grid::MultiLevelGridMap< maps::grid::SurfacePatchBase > MLGrid;
    
protected:
    
    typedef MLGrid::CellType Cell;
    typedef MLGrid::View View;
    typedef View::CellType ViewCell;
    typedef MLGrid::PatchType Patch;
    
    
    boost::shared_ptr<MLGrid> mlsGrid;
    bool addInitialPatch;
    Eigen::Affine3d initialPatch2Mls;
    double patchRadius;
    
    maps::grid::TraversabilityMap3d<NODE_TYPE*> trMap;
    int currentNodeId = 0; //used while expanding
    
    bool computePlaneRansac(NODE_TYPE &node);
    double computeSlope(const Eigen::Hyperplane< double, int(3) >& plane) const;
    Eigen::Vector3d computeSlopeDirection(const Eigen::Hyperplane< double, int(3) >& plane) const;
    
    bool checkForObstacles(NODE_TYPE* node);
    
    /** @return false if no allowed orientation was found (e.g. due to extreme slope)*/
    bool computeAllowedOrientations(NODE_TYPE* node);
    
    bool checkForFrontier(NODE_TYPE* node);
    
    void addConnectedPatches(NODE_TYPE* node);

    bool getConnectedPatch(const maps::grid::Index& idx, double height, const Patch*& patch);
    
    double interpolate(double x, double x0, double y0, double x1, double y1) const;

    NODE_TYPE *findMatchingTraversabilityPatchAt(maps::grid::Index idx, const double curHeight) const;
    
    NODE_TYPE *createTraversabilityPatchAt(maps::grid::Index idx, const double curHeight);
    
    TraversabilityConfig config;
    
    void addInitialPatchToMLS();
    
    int intersections();
    
public:
    BaseMapGenerator(const TraversabilityConfig &config);

    virtual ~BaseMapGenerator();

    void clearTrMap();
    
    void setInitialPatch(const Eigen::Affine3d &ground2Mls, double patchRadius);
    
    virtual NODE_TYPE *generateStartNode(const Eigen::Vector3d &startPos) = 0;
    void expandAll(const Eigen::Vector3d &startPos);

    void expandAll(NODE_TYPE *startNode);

    virtual bool expandNode(NODE_TYPE *node) = 0;
    
    void setMLSGrid(boost::shared_ptr<MLGrid> &grid);
    
    /**Returns the number of nodes after expansion*/
    int getNumNodes() const;
    
    const maps::grid::TraversabilityMap3d<NODE_TYPE *> &getTraversabilityMap() const;

    maps::grid::TraversabilityMap3d<NODE_TYPE* > getTraversabilityBaseMap() const;
        
    void setConfig(const TraversabilityConfig &config);

};

}
