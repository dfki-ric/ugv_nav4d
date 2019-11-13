#pragma once
#include <QObject>

#ifndef Q_MOC_RUN
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/TrajectoryVisualization.hpp>
#include <vizkit3d/MLSMapVisualization.hpp>
#include <vizkit3d/TraversabilityMap3dVisualization.hpp>
#include <vizkit3d/MotionPlanningLibrariesSbplSplineVisualization.hpp>
#include <vizkit3d/RigidBodyStateVisualization.hpp>
#include <maps/grid/MLSMap.hpp>
#include <base/Eigen.hpp>
#include <ugv_nav4d/Planner.hpp>
#include <ugv_nav4d/FrontierGenerator.hpp>
#include <ugv_nav4d/Config.hpp>
#endif



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
    void expandPressed();
    void slopeMetricComboBoxIndexChanged(int index);
    void parallelismCheckBoxStateChanged(int);
    void obstacleDistanceSpinBoxEditingFinished();
    void obstacleFactorSpinBoxEditingFinished();
    
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
    QWidget window;
    vizkit3d::MotionPlanningLibrariesSbplSplineVisualization splineViz;
    vizkit3d::TrajectoryVisualization trajViz;
    vizkit3d::TrajectoryVisualization trajViz2;
    vizkit3d::MLSMapVisualization mlsViz;
    vizkit3d::TraversabilityMap3dVisualization trav3dViz;
    vizkit3d::TraversabilityMap3dVisualization obstacleMapViz;
    vizkit3d::RigidBodyStateVisualization startViz;
    vizkit3d::RigidBodyStateVisualization goalViz;
    maps::grid::MLSMapKalman mlsMap;
    base::Pose start;
    base::Pose goal;
    base::Vector3d frontier;
    bool pickStart = true;
    bool startPicked = false;
    bool goalPicked = false;
    bool threadRunning = false;
    sbpl_spline_primitives::SplinePrimitivesConfig config;
    ugv_nav4d::Mobility mobility;
    ugv_nav4d::TraversabilityConfig conf;
    ugv_nav4d::PlannerConfig plannerConf;
    std::shared_ptr<ugv_nav4d::Planner> planner; //is pointer cause of lazy init
    std::vector<base::Trajectory> path;
    std::vector<base::Trajectory> beautifiedPath;
    std::shared_ptr<ugv_nav4d::FrontierGenerator> frontierGenerator;
    
};
