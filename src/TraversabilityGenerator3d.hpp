#pragma once

#include <maps/grid/MLSMap.hpp>
#include <boost/shared_ptr.hpp>

#include <maps/grid/TraversabilityMap3d.hpp>

class TraversabilityGenerator3d
{
public:
    class TrackingData
    {
    public:
        Eigen::Hyperplane<double, 3> plane;
        
    };
    
    class Config
    {
    public:
        Config(): maxStepHeight(0), maxSlope(0), robotHeight(0), robotSizeX(0), maxGapSize(0), numTraversabilityClasses(0), numNominalMeasurements(1), outliertFilterMinMeasurements(0), outliertFilterMaxStdDev(0.0), gridResolution(0.0) {};
        double maxStepHeight;
        double maxSlope;
        double robotHeight;
        double robotSizeX;
        double maxGapSize;
        int numTraversabilityClasses;
        /**
         * The amount of measurements a MLS-Patch needs
         * to get a probability of 1.0
         * */
        int numNominalMeasurements;
        
        int outliertFilterMinMeasurements;
        double outliertFilterMaxStdDev;
        double gridResolution;
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
    
    Config config;
public:
    TraversabilityGenerator3d(const Config &config);
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

