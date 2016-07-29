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
    
    
    typedef maps::grid::LevelList<maps::grid::SurfacePatchBase> Cell;
    typedef maps::grid::LevelList<maps::grid::SurfacePatchBase *> ViewCell;

    typedef maps::grid::MultiLevelGridMap< maps::grid::SurfacePatchBase >::View View;
    
    boost::shared_ptr<maps::grid::MultiLevelGridMap<maps::grid::SurfacePatchBase> > mlsGrid;
    maps::grid::TraversabilityMap3d<Node *> trMap;
    
    
    bool computePlaneRansac(maps::grid::TraversabilityNode<TrackingData> &node, const maps::grid::MultiLevelGridMap<maps::grid::SurfacePatchBase>::View &area);
    
    bool computePlane(maps::grid::TraversabilityNode<TrackingData> &node, const maps::grid::MultiLevelGridMap<maps::grid::SurfacePatchBase>::View &area);
    
    bool checkForObstacles(const maps::grid::MultiLevelGridMap< maps::grid::SurfacePatchBase >::View& area, maps::grid::TraversabilityNode< TraversabilityGenerator3d::TrackingData >* node);
    
    
    void addConnectedPatches(maps::grid::TraversabilityNode< TraversabilityGenerator3d::TrackingData >* node);

    bool getConnectedPatch(const maps::grid::Index& idx, double height, const maps::grid::SurfacePatchBase*& patch);
  
    bool updateDistToStart(double newValue, maps::grid::TraversabilityNodeBase* node);
    
    void clearTrMap();
    
    TraversabilityConfig config;
public:
    TraversabilityGenerator3d(const TraversabilityConfig &config);
    ~TraversabilityGenerator3d();

    maps::grid::TraversabilityNode<TrackingData> *generateStartNode(const Eigen::Vector3d &startPosWorld);
    void expandAll(const Eigen::Vector3d &startPosWorld);

    void expandAll(maps::grid::TraversabilityNode<TrackingData> *startNode);

    bool expandNode(maps::grid::TraversabilityNode< TraversabilityGenerator3d::TrackingData >* node);
    
    void setMLSGrid(boost::shared_ptr<maps::grid::MultiLevelGridMap<maps::grid::SurfacePatchBase> > &grid);
    
    const maps::grid::TraversabilityMap3d<Node *> &getTraversabilityMap() const;

    maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase* > getTraversabilityBaseMap() const;
    
protected:
    int intersections();
};

}