#pragma once

#include <maps/grid/MLSMap.hpp>
#include <boost/shared_ptr.hpp>
#include "TraversabilityConfig.hpp"
#include "TravGenNode.hpp"

#define GENERATE_DEBUG_DATA
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
    
private:
    
    typedef maps::grid::MultiLevelGridMap< maps::grid::SurfacePatchBase > MLGrid;
    typedef MLGrid::CellType Cell;
    typedef MLGrid::View View;
    typedef View::CellType ViewCell;
    typedef MLGrid::PatchType Patch;
    
    
    boost::shared_ptr<MLGrid > mlsGrid;
    maps::grid::TraversabilityMap3d<TravGenNode*> trMap;
    int currentNodeId = 0; //used while expanding
    
    bool computePlaneRansac(TravGenNode &node, const View &area);
    
    bool computePlane(TravGenNode &node, const View &area);
    
    double computeSlope(const Eigen::Hyperplane< double, int(3) >& plane) const;
    Eigen::Vector3d computeSlopeDirection(const Eigen::Hyperplane< double, int(3) >& plane) const;
    
    bool checkForObstacles(const View& area, TravGenNode* node);
    
    
    void addConnectedPatches(TravGenNode* node);

    bool getConnectedPatch(const maps::grid::Index& idx, double height, const Patch*& patch);
    
    
    TraversabilityConfig config;
public:
    TraversabilityGenerator3d(const TraversabilityConfig &config);

    ~TraversabilityGenerator3d();

    void clearTrMap();
    
    TravGenNode *generateStartNode(const Eigen::Vector3d &startPosWorld);
    void expandAll(const Eigen::Vector3d &startPosWorld);

    void expandAll(TravGenNode *startNode);

    bool expandNode(TravGenNode *node);
    
    void setMLSGrid(boost::shared_ptr<MLGrid> &grid);
    
    /**Returns the number of nodes after expansion*/
    int getNumNodes() const;
    
    const maps::grid::TraversabilityMap3d<TravGenNode *> &getTraversabilityMap() const;

    maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase* > getTraversabilityBaseMap() const;
        
    void setConfig(const TraversabilityConfig &config);
    
protected:
    int intersections();
};

}
