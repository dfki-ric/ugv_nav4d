#pragma once
#include <QObject>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/TrajectoryVisualization.hpp>
#include <vizkit3d/MLSMapVisualization.hpp>
#include <vizkit3d/TraversabilityMap3dVisualization.hpp>
#include <vizkit3d/EnvironmentXYZThetaVisualization.hpp>
#include <vizkit3d/MotionPlanningLibrariesSbplMprimsVisualization.hpp>
#include <vizkit3d/MotionPlanningLibrariesSbplSplineVisualization.hpp>
#include <vizkit3d/RigidBodyStateVisualization.hpp>
#include <maps/grid/MLSMap.hpp>
#include <base/Eigen.hpp>
#include "Planner.hpp"

class PlannerGui : public QObject
{
    Q_OBJECT;
    
public:
    PlannerGui(int argc, char** argv);
    
    void show();
public slots:
    /** Called when the user clicks a patch on the mls */
    void picked(float x, float y, float z);
    
    //display the planner results
    void plannerIsDone();
    
    /**plan from @p staro to @p goal */
    void plan(const base::Pose& start, const base::Pose& goal);
    
signals:
    //is emitted if the planner thread is done
    void plannerDone();
    
private slots:
    void maxSlopeEditingFinished();
    void inclineLimittingLimitSpinBoxEditingFinished();
    void inclineLimittingMinSlopeSpinBoxEditingFinished();
    void slopeMetricScaleSpinBoxEditingFinished();
    void startOrientationChanged(int newValue);
    void goalOrientationChanged(int newValue);
    void timeEditingFinished();
    void replanButtonReleased();
    void planFrontierButtonReleased();
    void expandPressed();
    void slopeMetricComboBoxIndexChanged(int index);
    void heuristicComboBoxIndexChanged(int index);
    void parallelismCheckBoxStateChanged(int);
    void obstacleDistanceSpinBoxEditingFinished();
    void obstacleFactorSpinBoxEditingFinished();
    void frontierXEditFinished(double value);
    void frontierYEditFinished(double value);
    void frontierZEditFinished(double value);
    
private:
    void loadMls();
    void loadMls(const std::string& path);
    void startPlanThread();
    
private:
    vizkit3d::Vizkit3DWidget* widget;
    QDoubleSpinBox* maxSlopeSpinBox;
    QDoubleSpinBox* slopeMetricScaleSpinBox;
    QDoubleSpinBox* time;
    QDoubleSpinBox* inclineLimittingMinSlopeSpinBox;
    QDoubleSpinBox* inclineLimittingLimitSpinBox;
    QSlider* startOrientatationSlider;
    QSlider* goalOrientationSlider;
    QDoubleSpinBox* obstacleDistanceSpinBox;
    QDoubleSpinBox* obstacleFactorSpinBox;
    QPushButton* expandButton;
    QComboBox* slopeMetricComboBox;
    QComboBox* heuristicComboBox;
    QCheckBox* parallelismCheckBox;
    QProgressBar* bar;
    QDoubleSpinBox* frontierX;
    QDoubleSpinBox* frontierY;
    QDoubleSpinBox* frontierZ;
    QWidget window;
    vizkit3d::MotionPlanningLibrariesSbplSplineVisualization splineViz;
    vizkit3d::TrajectoryVisualization trajViz;
    vizkit3d::MLSMapVisualization mlsViz;
    vizkit3d::TraversabilityMap3dVisualization trav3dViz;
    vizkit3d::EnvironmentXYZThetaVisualization envViz;
    vizkit3d::RigidBodyStateVisualization startViz;
    vizkit3d::RigidBodyStateVisualization goalViz;
    maps::grid::MLSMapKalman mlsMap;
    base::Pose start;
    base::Pose goal;
    base::Vector3d frontier;
    bool pickStart = true;
    bool threadRunning = false;
    motion_planning_libraries::SplinePrimitivesConfig config;
    motion_planning_libraries::Mobility mobility;
    ugv_nav4d::TraversabilityConfig conf;
    std::shared_ptr<ugv_nav4d::Planner> planner; //is pointer cause of lazy init
    std::vector<base::Trajectory> path;
    
};