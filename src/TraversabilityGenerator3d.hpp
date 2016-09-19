#pragma once

#include <maps/grid/MLSMap.hpp>
#include <boost/shared_ptr.hpp>

#include <maps/grid/TraversabilityMap3d.hpp>
#include "TraversabilityConfig.hpp"

namespace ugv_nav4d
{


class TraversabilityGenerator3d
{
public:
    class TrackingData
    {
    public:
        Eigen::Hyperplane<double, 3> plane;
        
    };
    
    typedef maps::grid::TraversabilityNode<TrackingData> Node;

private:
    
    typedef maps::grid::MultiLevelGridMap< maps::grid::SurfacePatchBase > MLGrid;
    typedef MLGrid::CellType Cell;
    typedef MLGrid::View View;
    typedef View::CellType ViewCell;
    typedef MLGrid::PatchType Patch;
    
    
    boost::shared_ptr<MLGrid > mlsGrid;
    maps::grid::TraversabilityMap3d<Node *> trMap;
    
    
    bool computePlaneRansac(Node &node, const View &area);
    
    bool computePlane(Node &node, const View &area);
    
    bool checkForObstacles(const View& area, Node* node);
    
    
    void addConnectedPatches(Node* node);

    bool getConnectedPatch(const maps::grid::Index& idx, double height, const Patch*& patch);
  
    bool updateDistToStart(double newValue, maps::grid::TraversabilityNodeBase* node);
    
    void clearTrMap();
    
    TraversabilityConfig config;
public:
    TraversabilityGenerator3d(const TraversabilityConfig &config);
    ~TraversabilityGenerator3d();

    Node *generateStartNode(const Eigen::Vector3d &startPosWorld);
    void expandAll(const Eigen::Vector3d &startPosWorld);

    void expandAll(Node *startNode);

    bool expandNode(Node *node);
    
    void setMLSGrid(boost::shared_ptr<MLGrid> &grid);
    
    const maps::grid::TraversabilityMap3d<Node *> &getTraversabilityMap() const;

    maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase* > getTraversabilityBaseMap() const;
    
protected:
    int intersections();
};

}
