#pragma once
#include <QObject>
#include <QWidget>
#include <atomic> 
#include <thread>
#include <functional>

class QPlainTextEdit;

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
class QCheckBox;
class QLabel;
class QGroupBox;

namespace vizkit3d {
    class Vizkit3DWidget;
}

class PlannerGui : public QObject
{
    Q_OBJECT;

public:
    PlannerGui(int argc, char** argv, bool autoLoadMls = true, bool loadConfigFromFile = true);
    PlannerGui(const std::string &dumpName);
    ~PlannerGui();

    void show();
    void setupPlanner(int argc, char** argv, bool autoLoadMls = true, bool loadConfigFromFile = true);
    void setupUI();
    void setupDefaultConfigs();

    std::function<void(const base::Pose& start, const base::Pose& goal)> customPlanCallback;
    // Invoked when the user clicks "Update Params"; lets an external owner (e.g. the ROS 2 node)
    // read the updated config structs and apply them.
    std::function<void()> configUpdateCallback;
    void updateMlsMap(const maps::grid::MLSMapSloped& map);
    void updateTravMap(const traversability_generator3d::TravMap3d& map);
    // Set the displayed start pose (RigidBodyState viz) from an external source, e.g. the robot
    // pose held by the ROS node. Used when the GUI does not pick the start itself.
    void updateStartPose(const base::Pose& startPose);
    // Update the status label from an external source (e.g. the ROS node's map/planner state).
    void setStatusMessage(const std::string& msg);
    void showPath(const std::vector<trajectory_follower::SubTrajectory>& path2D,
                  const std::vector<trajectory_follower::SubTrajectory>& path3D,
                  ugv_nav4d::Planner::PLANNING_RESULT result);

    // Config structures - accessible for reading/setting from node
    sbpl_spline_primitives::SplinePrimitivesConfig splineConfig;
    ugv_nav4d::Mobility mobilityConfig;
    traversability_generator3d::TraversabilityConfig travConfig;
    ugv_nav4d::PlannerConfig plannerConfig;
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
    void logReceived(const QString& text);
    //emitted from the planning worker thread to update the status label per phase
    void statusUpdate(const QString& text, const QString& color);

private slots:
    void onStatusUpdate(const QString& text, const QString& color);
    void appendLog(const QString& text);
    void clearLogReleased();
    void maxSlopeEditingFinished();
    void inclineLimittingLimitSpinBoxEditingFinished();
    void inclineLimittingMinSlopeSpinBoxEditingFinished();
    void slopeMetricScaleSpinBoxEditingFinished();
    void startOrientationChanged(int newValue);
    void goalOrientationChanged(int newValue);
    void timeEditingFinished();
    void replanButtonReleased();
    void updateParamsButtonReleased();
    void dumpPressed();
    void slopeMetricComboBoxIndexChanged(int index);
    void numThreadsValueChanged(int newValue);
    void obstacleDistanceSpinBoxEditingFinished();
    void robotSizeXEditingFinished();
    void robotSizeYEditingFinished();
    void robotHeightEditingFinished();
    void distToGroundEditingFinished();
    void translationSpeedEditingFinished();
    void rotationSpeedEditingFinished();
    void minTurningRadiusEditingFinished();

    // Spline Slots
    void splineGridSizeEditingFinished();
    void splineNumAnglesValueChanged(int value);
    void splineNumEndAnglesValueChanged(int value);
    void splineDestCircleRadiusEditingFinished();
    void splineCellSkipFactorEditingFinished();
    void splineGenPointTurnMotionsStateChanged(int state);
    void splineGenLateralMotionsStateChanged(int state);
    void splineGenBackwardMotionsStateChanged(int state);
    void splineGenForwardMotionsStateChanged(int state);
    void splineOrderValueChanged(int value);

    // Mobility Slots
    void mobSearchRadiusEditingFinished();
    void mobSearchProgressStepsEditingFinished();
    void mobMultForwardValueChanged(int value);
    void mobMultBackwardValueChanged(int value);
    void mobMultLateralValueChanged(int value);
    void mobMultForwardTurnValueChanged(int value);
    void mobMultBackwardTurnValueChanged(int value);
    void mobMultPointTurnValueChanged(int value);
    void mobMultLateralCurveValueChanged(int value);
    void mobMaxMotionCurveLengthEditingFinished();
    void mobSplineSamplingResEditingFinished();
    void mobRemoveGoalOffsetStateChanged(int state);
    void mobCurvaturePenaltyWeightEditingFinished();
    void mobAngularCostWeightEditingFinished();

    // Traversability Slots
    void travGridResolutionEditingFinished();
    void travMaxStepHeightEditingFinished();
    void travMinTraversablePercentageEditingFinished();
    void travAllowForwardDownhillStateChanged(int state);
    void travEnableInclineLimittingStateChanged(int state);
    void travArticulatedSuspensionStateChanged(int state);
    void travObstacleInflationMultiplierEditingFinished();
    void travPartiallyTraversableMultiplierEditingFinished();
    void travNumYawSamplesValueChanged(int value);


    // Planner Slots
    void planEpsilonStepsEditingFinished();
    void planInitialEpsilonEditingFinished();
    void planUsePathStatisticsStateChanged(int state);
    void planSearchUntilFirstSolutionStateChanged(int state);
    void planCorridorWidthEditingFinished();
    void planGoalOrientationMarginEditingFinished();
    void planGoalDistanceMarginEditingFinished();
    
private:
    void loadMls();
    void loadMls(const std::string& path);
    void startPlanThread();
    void updateWidgetValues();
    // Grid size (spline) and grid resolution (traversability) must stay equal; this keeps both
    // config fields and both spinboxes in sync and rebuilds the planner with new primitives.
    void applyGridResolution(double res);
    
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
    QComboBox* slopeMetricComboBox;
    QComboBox* heuristicComboBox;
    QSpinBox* numThreadsSpinBox;
    QDoubleSpinBox* robotSizeXSpinBox;
    QDoubleSpinBox* robotSizeYSpinBox;
    QDoubleSpinBox* robotHeightSpinBox;
    QDoubleSpinBox* distToGroundSpinBox;
    QDoubleSpinBox* translationSpeedSpinBox;
    QDoubleSpinBox* rotationSpeedSpinBox;
    QDoubleSpinBox* minTurningRadiusSpinBox;

    // Spline Config Widgets
    QDoubleSpinBox* splineGridSizeSpinBox;
    QSpinBox* splineNumAnglesSpinBox;
    QSpinBox* splineNumEndAnglesSpinBox;
    QDoubleSpinBox* splineDestCircleRadiusSpinBox;
    QDoubleSpinBox* splineCellSkipFactorSpinBox;
    QCheckBox* splineGenPointTurnMotionsCheckBox;
    QCheckBox* splineGenLateralMotionsCheckBox;
    QCheckBox* splineGenBackwardMotionsCheckBox;
    QCheckBox* splineGenForwardMotionsCheckBox;
    QSpinBox* splineOrderSpinBox;

    // Mobility Config Widgets
    QDoubleSpinBox* mobSearchRadiusSpinBox;
    QDoubleSpinBox* mobSearchProgressStepsSpinBox;
    QSpinBox* mobMultForwardSpinBox;
    QSpinBox* mobMultBackwardSpinBox;
    QSpinBox* mobMultLateralSpinBox;
    QSpinBox* mobMultForwardTurnSpinBox;
    QSpinBox* mobMultBackwardTurnSpinBox;
    QSpinBox* mobMultPointTurnSpinBox;
    QSpinBox* mobMultLateralCurveSpinBox;
    QDoubleSpinBox* mobMaxMotionCurveLengthSpinBox;
    QDoubleSpinBox* mobSplineSamplingResSpinBox;
    QCheckBox* mobRemoveGoalOffsetCheckBox;
    QDoubleSpinBox* mobCurvaturePenaltyWeightSpinBox;
    QDoubleSpinBox* mobAngularCostWeightSpinBox;

    // Traversability Config Widgets
    QDoubleSpinBox* travGridResolutionSpinBox;
    QDoubleSpinBox* travMaxStepHeightSpinBox;
    QDoubleSpinBox* travMinTraversablePercentageSpinBox;
    QCheckBox* travAllowForwardDownhillCheckBox;
    QCheckBox* travEnableInclineLimittingCheckBox;
    QCheckBox* travArticulatedSuspensionCheckBox;
    QDoubleSpinBox* travObstacleInflationMultiplierSpinBox;
    QDoubleSpinBox* travPartiallyTraversableMultiplierSpinBox;
    QSpinBox* travNumYawSamplesSpinBox;


    // Planner Config Widgets
    QDoubleSpinBox* planEpsilonStepsSpinBox;
    QDoubleSpinBox* planInitialEpsilonSpinBox;
    QCheckBox* planUsePathStatisticsCheckBox;
    QCheckBox* planSearchUntilFirstSolutionCheckBox;
    QDoubleSpinBox* planCorridorWidthSpinBox;
    QDoubleSpinBox* planGoalOrientationMarginSpinBox;
    QDoubleSpinBox* planGoalDistanceMarginSpinBox;
    
    // Height (Z) filter applied to imported PLY/point clouds before building the MLS.
    QCheckBox* heightFilterCheckBox;
    QDoubleSpinBox* heightFilterMinSpinBox;
    QDoubleSpinBox* heightFilterMaxSpinBox;

    QProgressBar* bar;
    QLabel* statusLabel;
    QPlainTextEdit* logConsole;
    int pipeFd[2];
    int originalStdout;
    int originalStderr;
    std::thread logReaderThread;
    std::atomic<bool> stopLogReader{false};
    ugv_nav4d::Planner::PLANNING_RESULT lastPlanningResult;
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
    bool plannerHasRun = false;
    std::shared_ptr<ugv_nav4d::Planner> planner; //is pointer cause of lazy init
    std::vector<trajectory_follower::SubTrajectory> path;
    std::vector<trajectory_follower::SubTrajectory> beautifiedPath;    
    std::shared_ptr<traversability_generator3d::TraversabilityGenerator3d> travGen;
};
