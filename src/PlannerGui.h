#pragma once
#include <QObject>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/TrajectoryVisualization.hpp>
#include <vizkit3d/MLSMapVisualization.hpp>
#include <vizkit3d/TraversabilityMap3dVisualization.hpp>
#include <vizkit3d/EnvironmentXYZThetaVisualization.hpp>
#include <vizkit3d/MotionPlanningLibrariesSbplMprimsVisualization.hpp>
#include <vizkit3d/MotionPlanningLibrariesSbplSplineVisualization.hpp>
#include <maps/grid/MLSMap.hpp>
#include <base/Eigen.hpp>
#include "Planner.hpp"

class PlannerGui : public QObject
{
    Q_OBJECT;
    
public:
    PlannerGui(int argc, char** argv);
    
    void exec();
public slots:
    /** Called when the user clicks a patch on the mls */
    void picked(float x, float y, float z);
    
    //display the planner results
    void plannerIsDone();
    
    /**plan from @p staro to @p goal */
    void plan(const Eigen::Vector3f& start, const Eigen::Vector3f& goal);
    
signals:
    //is emitted if the planner thread is done
    void plannerDone();
    
private slots:
    void slopeMetricEditingFinished();
    void replanButtonReleased();
    void expandPressed();
    
private:
    void loadMls();
    void loadMls(const std::string& path);
    void startPlanThread();
    
private:
    QApplication app;
    vizkit3d::Vizkit3DWidget widget;
    QDoubleSpinBox* slopeMetricSpinBox;
    QWidget window;
    vizkit3d::MotionPlanningLibrariesSbplSplineVisualization splineViz;
    vizkit3d::TrajectoryVisualization trajViz;
    vizkit3d::MLSMapVisualization mlsViz;
    vizkit3d::TraversabilityMap3dVisualization trav3dViz;
    vizkit3d::EnvironmentXYZThetaVisualization envViz;
    maps::grid::MLSMapPrecalculated mlsMap;
    Eigen::Vector3f start; //is inf if not set
    Eigen::Vector3f goal; //is inf if not set
    bool pickStart = true;
    bool threadRunning = false;
    motion_planning_libraries::SplinePrimitivesConfig config;
    motion_planning_libraries::Mobility mobility;
    ugv_nav4d::TraversabilityConfig conf;
    std::shared_ptr<ugv_nav4d::Planner> planner; //is pointer cause of lazy init
    
};