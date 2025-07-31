#pragma once
#include <QObject>
#include <QWidget>
#include <atomic> 

#ifndef Q_MOC_RUN
#include <vizkit3d/SubTrajectoryVisualization.hpp>
#include <vizkit3d/MLSMapVisualization.hpp>
#include <vizkit3d/TravMap3dVisualization.hpp>
#include <vizkit3d/GridVisualization.hpp>
#include <vizkit3d/SbplSplineVisualization.hpp>
#include <vizkit3d/RigidBodyStateVisualization.hpp>
#include <maps/grid/MLSMap.hpp>
#include <base/Eigen.hpp>
#include <ugv_nav4d/Planner.hpp>
#include <traversability_generator3d/TraversabilityConfig.hpp>
#include <traversability_generator3d/TraversabilityGenerator3d.hpp>
#include <trajectory_follower/SubTrajectory.hpp>
#endif

class QDoubleSpinBox;
class QSpinBox;
class QSlider;
class QPushButton;
class QComboBox;
class QProgressBar;

namespace vizkit3d {
    class Vizkit3DWidget;
}

class PlannerGui : public QObject
{
    Q_OBJECT;
    
    void setupPlanner(int argc, char** argv);
    void setupUI();
    
public:
    PlannerGui(int argc, char** argv);
    PlannerGui(const std::string &dumpName);
    
    void show();
public slots:
    /** Called when the user clicks a patch on the mls */
    void picked(float x, float y,float z, int buttonMask, int modifierMask);
    
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
    void dumpPressed();
    void slopeMetricComboBoxIndexChanged(int index);
    void numThreadsValueChanged(int newValue);
    void obstacleDistanceSpinBoxEditingFinished();
    void obstacleFactorSpinBoxEditingFinished();
    
private:
    void loadMls();
    void loadMls(const std::string& path);
    void startPlanThread();
    
private:

    std::atomic<bool> inplanningphase{false}; // Atomic for thread-safe flag
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
    QComboBox* slopeMetricComboBox;
    QComboBox* heuristicComboBox;
    QSpinBox* numThreadsSpinBox;
    QProgressBar* bar;
    QWidget window;
    vizkit3d::SbplSplineVisualization splineViz;
    vizkit3d::SubTrajectoryVisualization trajViz;
    vizkit3d::SubTrajectoryVisualization trajViz2;
    vizkit3d::MLSMapVisualization mlsViz;
    vizkit3d::TravMap3dVisualization trav3dViz;
    vizkit3d::RigidBodyStateVisualization startViz;
    vizkit3d::RigidBodyStateVisualization goalViz;
    vizkit3d::GridVisualization gridViz;
    maps::grid::MLSMapSloped mlsMap;
    base::Pose start;
    base::Pose goal;
    bool pickStart = true;
    bool startPicked = false;
    bool goalPicked = false;
    bool threadRunning = false;
    bool usingPlannerDump = false;
    sbpl_spline_primitives::SplinePrimitivesConfig splineConfig;
    ugv_nav4d::Mobility mobilityConfig;
    traversability_generator3d::TraversabilityConfig travConfig;
    ugv_nav4d::PlannerConfig plannerConfig;
    std::shared_ptr<ugv_nav4d::Planner> planner; //is pointer cause of lazy init
    std::vector<trajectory_follower::SubTrajectory> path;
    std::vector<trajectory_follower::SubTrajectory> beautifiedPath;    
    std::shared_ptr<traversability_generator3d::TraversabilityGenerator3d> travGen;
};
