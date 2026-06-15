#include "PlannerGui.h"
#include <QFileDialog>
#include <QSpinBox>
#include <QPushButton>
#include <QProgressBar>
#include <QLabel>
#include <QSlider>
#include <QComboBox>
#include <QHBoxLayout>
#include <QTabWidget>
#include <QFormLayout>
#include <QCheckBox>
#include <thread>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <ugv_nav4d/PreComputedMotions.hpp>
#include <boost/filesystem.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <ugv_nav4d/PlannerDump.hpp>
#include <pcl/common/transforms.h>
#include <base-logging/Logging.hpp>
#include <ugv_nav4d/ConfigLoader.hpp>

#ifdef ENABLE_DEBUG_DRAWINGS
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>
#include <vizkit3d_debug_drawings/DebugDrawingColors.hpp>
#endif

using namespace ugv_nav4d;

PlannerGui::PlannerGui(const std::string& dumpName): QObject()
{
    setupUI();
    
    PlannerDump dump(dumpName);

    mobilityConfig = dump.getMobilityConf();
    travConfig = dump.getTravConfig();
    splineConfig = dump.getSplineConfig();
    plannerConfig = dump.getPlannerConfig();
    
    planner.reset(new ugv_nav4d::Planner(splineConfig, travConfig, mobilityConfig, plannerConfig));
    travGen.reset(new traversability_generator3d::TraversabilityGenerator3d(travConfig));

    sbpl_spline_primitives::SbplSplineMotionPrimitives primitives(splineConfig);
    splineViz.setMaxCurvature(ugv_nav4d::PreComputedMotions::calculateCurvatureFromRadius(mobilityConfig.minTurningRadius));
    splineViz.updateData(primitives);

    start = dump.getStart().getPose();
    goal = dump.getGoal().getPose();

    startViz.updateData(dump.getStart());
    goalViz.updateData(dump.getGoal());
    planner->updateMap(dump.getTravMap());
    trav3dViz.updateData(*(planner->getTraversabilityMap()));
    inplanningphase = false;
    usingPlannerDump = true;
    updateWidgetValues();
}


PlannerGui::PlannerGui(int argc, char** argv): QObject()
{
    usingPlannerDump = false;
    setupUI();
    setupPlanner(argc, argv);
}

void PlannerGui::setupUI()
{
    start.orientation.setIdentity();
    goal.orientation.setIdentity();
    
    widget = new vizkit3d::Vizkit3DWidget();
#ifdef ENABLE_DEBUG_DRAWINGS
    V3DD::CONFIGURE_DEBUG_DRAWINGS_USE_EXISTING_WIDGET(widget);
#endif
    trav3dViz.setPluginName("TravMap");
    
    startViz.setPluginName("Start Pose");
    goalViz.setPluginName("Goal Pose");

    widget->setCameraManipulator(vizkit3d::ORBIT_MANIPULATOR);
    widget->addPlugin(&splineViz);
    widget->addPlugin(&trajViz);
    widget->addPlugin(&trajViz2);
    widget->addPlugin(&mlsViz);
    widget->addPlugin(&trav3dViz);
    widget->addPlugin(&startViz);
    widget->addPlugin(&goalViz);
    widget->addPlugin(&gridViz);
    
    splineViz.setPluginEnabled(false);
    splineViz.setPluginName("Splines");

    gridViz.setPluginEnabled(false);
    gridViz.setPluginName("Grid");

    mlsViz.setCycleHeightColor(true);
    mlsViz.setShowPatchExtents(false); 
    mlsViz.setShowNormals(false);
    mlsViz.setPluginName("MLSMap");
    
    trajViz.setLineWidth(5);
    trajViz.setColor(QColor("Cyan"));
    trajViz.setPluginEnabled(false);
    trajViz.setPluginName("Trajectory 2D");

    trajViz2.setLineWidth(5);
    trajViz2.setColor(QColor("magenta"));
    trajViz2.setPluginName("Trajectory 3D");

    QVBoxLayout* layout = new QVBoxLayout();
    layout->addWidget(widget);

    QFormLayout* orientationLayout = new QFormLayout();

    startOrientatationSlider = new QSlider(Qt::Horizontal);
    startOrientatationSlider->setMinimum(0);
    startOrientatationSlider->setMaximum(359);
    connect(startOrientatationSlider, SIGNAL(sliderMoved(int)), this, SLOT(startOrientationChanged(int)));
    orientationLayout->addRow("Start Orientation (deg):", startOrientatationSlider);

    goalOrientationSlider = new QSlider(Qt::Horizontal);
    goalOrientationSlider->setMinimum(0);
    goalOrientationSlider->setMaximum(359);
    connect(goalOrientationSlider, SIGNAL(sliderMoved(int)), this, SLOT(goalOrientationChanged(int)));
    orientationLayout->addRow("Goal Orientation (deg):", goalOrientationSlider);

    layout->addLayout(orientationLayout);

    QTabWidget* tabWidget = new QTabWidget();

    // Tab 1: UGV/Mobility
    QWidget* robotTab = new QWidget();
    QFormLayout* robotFormLayout = new QFormLayout();

    translationSpeedSpinBox = new QDoubleSpinBox();
    translationSpeedSpinBox->setMinimum(0.01);
    translationSpeedSpinBox->setMaximum(20.0);
    translationSpeedSpinBox->setSingleStep(0.05);
    translationSpeedSpinBox->setDecimals(2);
    connect(translationSpeedSpinBox, SIGNAL(editingFinished()), this, SLOT(translationSpeedEditingFinished()));
    robotFormLayout->addRow("Translation Speed (m/s):", translationSpeedSpinBox);

    rotationSpeedSpinBox = new QDoubleSpinBox();
    rotationSpeedSpinBox->setMinimum(0.01);
    rotationSpeedSpinBox->setMaximum(20.0);
    rotationSpeedSpinBox->setSingleStep(0.05);
    rotationSpeedSpinBox->setDecimals(2);
    connect(rotationSpeedSpinBox, SIGNAL(editingFinished()), this, SLOT(rotationSpeedEditingFinished()));
    robotFormLayout->addRow("Rotation Speed (rad/s):", rotationSpeedSpinBox);

    minTurningRadiusSpinBox = new QDoubleSpinBox();
    minTurningRadiusSpinBox->setMinimum(0.0);
    minTurningRadiusSpinBox->setMaximum(50.0);
    minTurningRadiusSpinBox->setSingleStep(0.05);
    minTurningRadiusSpinBox->setDecimals(2);
    connect(minTurningRadiusSpinBox, SIGNAL(editingFinished()), this, SLOT(minTurningRadiusEditingFinished()));
    robotFormLayout->addRow("Min Turning Radius (m):", minTurningRadiusSpinBox);

    robotSizeXSpinBox = new QDoubleSpinBox();
    robotSizeXSpinBox->setMinimum(0.05);
    robotSizeXSpinBox->setMaximum(20.0);
    robotSizeXSpinBox->setSingleStep(0.05);
    robotSizeXSpinBox->setDecimals(2);
    connect(robotSizeXSpinBox, SIGNAL(editingFinished()), this, SLOT(robotSizeXEditingFinished()));
    robotFormLayout->addRow("Robot Size X (m):", robotSizeXSpinBox);

    robotSizeYSpinBox = new QDoubleSpinBox();
    robotSizeYSpinBox->setMinimum(0.05);
    robotSizeYSpinBox->setMaximum(20.0);
    robotSizeYSpinBox->setSingleStep(0.05);
    robotSizeYSpinBox->setDecimals(2);
    connect(robotSizeYSpinBox, SIGNAL(editingFinished()), this, SLOT(robotSizeYEditingFinished()));
    robotFormLayout->addRow("Robot Size Y (m):", robotSizeYSpinBox);

    robotHeightSpinBox = new QDoubleSpinBox();
    robotHeightSpinBox->setMinimum(0.05);
    robotHeightSpinBox->setMaximum(20.0);
    robotHeightSpinBox->setSingleStep(0.05);
    robotHeightSpinBox->setDecimals(2);
    connect(robotHeightSpinBox, SIGNAL(editingFinished()), this, SLOT(robotHeightEditingFinished()));
    robotFormLayout->addRow("Robot Height (m):", robotHeightSpinBox);

    distToGroundSpinBox = new QDoubleSpinBox();
    distToGroundSpinBox->setMinimum(0.0);
    distToGroundSpinBox->setMaximum(5.0);
    distToGroundSpinBox->setSingleStep(0.01);
    distToGroundSpinBox->setDecimals(3);
    connect(distToGroundSpinBox, SIGNAL(editingFinished()), this, SLOT(distToGroundEditingFinished()));
    robotFormLayout->addRow("Distance to Ground (m):", distToGroundSpinBox);

    mobSearchRadiusSpinBox = new QDoubleSpinBox();
    mobSearchRadiusSpinBox->setMinimum(0.0);
    mobSearchRadiusSpinBox->setMaximum(20.0);
    mobSearchRadiusSpinBox->setSingleStep(0.05);
    mobSearchRadiusSpinBox->setDecimals(2);
    connect(mobSearchRadiusSpinBox, SIGNAL(editingFinished()), this, SLOT(mobSearchRadiusEditingFinished()));
    robotFormLayout->addRow("Search Radius (m):", mobSearchRadiusSpinBox);

    mobSearchProgressStepsSpinBox = new QDoubleSpinBox();
    mobSearchProgressStepsSpinBox->setMinimum(0.01);
    mobSearchProgressStepsSpinBox->setMaximum(5.0);
    mobSearchProgressStepsSpinBox->setSingleStep(0.05);
    mobSearchProgressStepsSpinBox->setDecimals(2);
    connect(mobSearchProgressStepsSpinBox, SIGNAL(editingFinished()), this, SLOT(mobSearchProgressStepsEditingFinished()));
    robotFormLayout->addRow("Search Progress Steps (m):", mobSearchProgressStepsSpinBox);

    mobMaxMotionCurveLengthSpinBox = new QDoubleSpinBox();
    mobMaxMotionCurveLengthSpinBox->setMinimum(1.0);
    mobMaxMotionCurveLengthSpinBox->setMaximum(1000.0);
    mobMaxMotionCurveLengthSpinBox->setSingleStep(1.0);
    mobMaxMotionCurveLengthSpinBox->setDecimals(1);
    connect(mobMaxMotionCurveLengthSpinBox, SIGNAL(editingFinished()), this, SLOT(mobMaxMotionCurveLengthEditingFinished()));
    robotFormLayout->addRow("Max Motion Curve Length:", mobMaxMotionCurveLengthSpinBox);

    mobSplineSamplingResSpinBox = new QDoubleSpinBox();
    mobSplineSamplingResSpinBox->setMinimum(0.001);
    mobSplineSamplingResSpinBox->setMaximum(1.0);
    mobSplineSamplingResSpinBox->setSingleStep(0.01);
    mobSplineSamplingResSpinBox->setDecimals(3);
    connect(mobSplineSamplingResSpinBox, SIGNAL(editingFinished()), this, SLOT(mobSplineSamplingResEditingFinished()));
    robotFormLayout->addRow("Spline Sampling Res (m):", mobSplineSamplingResSpinBox);

    mobRemoveGoalOffsetCheckBox = new QCheckBox();
    connect(mobRemoveGoalOffsetCheckBox, SIGNAL(stateChanged(int)), this, SLOT(mobRemoveGoalOffsetStateChanged(int)));
    robotFormLayout->addRow("Remove Goal Offset", mobRemoveGoalOffsetCheckBox);

    mobCurvaturePenaltyWeightSpinBox = new QDoubleSpinBox();
    mobCurvaturePenaltyWeightSpinBox->setMinimum(0.0);
    mobCurvaturePenaltyWeightSpinBox->setMaximum(100.0);
    mobCurvaturePenaltyWeightSpinBox->setSingleStep(0.05);
    mobCurvaturePenaltyWeightSpinBox->setDecimals(2);
    connect(mobCurvaturePenaltyWeightSpinBox, SIGNAL(editingFinished()), this, SLOT(mobCurvaturePenaltyWeightEditingFinished()));
    robotFormLayout->addRow("Curvature Penalty Weight:", mobCurvaturePenaltyWeightSpinBox);

    mobAngularCostWeightSpinBox = new QDoubleSpinBox();
    mobAngularCostWeightSpinBox->setMinimum(0.0);
    mobAngularCostWeightSpinBox->setMaximum(100.0);
    mobAngularCostWeightSpinBox->setSingleStep(0.05);
    mobAngularCostWeightSpinBox->setDecimals(2);
    connect(mobAngularCostWeightSpinBox, SIGNAL(editingFinished()), this, SLOT(mobAngularCostWeightEditingFinished()));
    robotFormLayout->addRow("Angular Cost Weight:", mobAngularCostWeightSpinBox);

    robotTab->setLayout(robotFormLayout);
    tabWidget->addTab(robotTab, "UGV/Mobility");

    // Tab 2: Traversability
    QWidget* travTab = new QWidget();
    QFormLayout* travFormLayout = new QFormLayout();

    maxSlopeSpinBox = new QDoubleSpinBox();
    maxSlopeSpinBox->setMinimum(1);
    maxSlopeSpinBox->setMaximum(60);
    maxSlopeSpinBox->setDecimals(2);
    connect(maxSlopeSpinBox, SIGNAL(editingFinished()), this, SLOT(maxSlopeEditingFinished()));
    travFormLayout->addRow("Max Slope (deg):", maxSlopeSpinBox);

    inclineLimittingMinSlopeSpinBox = new QDoubleSpinBox();
    inclineLimittingMinSlopeSpinBox->setMinimum(0.0);
    inclineLimittingMinSlopeSpinBox->setMaximum(180.0);
    inclineLimittingMinSlopeSpinBox->setDecimals(2);
    connect(inclineLimittingMinSlopeSpinBox, SIGNAL(editingFinished()), this, SLOT(inclineLimittingMinSlopeSpinBoxEditingFinished()));
    travFormLayout->addRow("Incline Limit Min Slope (deg):", inclineLimittingMinSlopeSpinBox);

    inclineLimittingLimitSpinBox = new QDoubleSpinBox();
    inclineLimittingLimitSpinBox->setMinimum(0.00001);
    inclineLimittingLimitSpinBox->setMaximum(90.0);
    inclineLimittingLimitSpinBox->setDecimals(2);
    connect(inclineLimittingLimitSpinBox, SIGNAL(editingFinished()), this, SLOT(inclineLimittingLimitSpinBoxEditingFinished()));
    travFormLayout->addRow("Incline Limit Max Slope (deg):", inclineLimittingLimitSpinBox);

    slopeMetricScaleSpinBox = new QDoubleSpinBox();
    slopeMetricScaleSpinBox->setMinimum(0.0);
    slopeMetricScaleSpinBox->setMaximum(999999.0);
    slopeMetricScaleSpinBox->setDecimals(2);
    connect(slopeMetricScaleSpinBox, SIGNAL(editingFinished()), this, SLOT(slopeMetricScaleSpinBoxEditingFinished()));
    travFormLayout->addRow("Slope Metric Scale:", slopeMetricScaleSpinBox);

    slopeMetricComboBox = new QComboBox();
    slopeMetricComboBox->addItem("NONE");
    slopeMetricComboBox->addItem("AVG_SLOPE");
    slopeMetricComboBox->addItem("MAX_SLOPE");
    slopeMetricComboBox->addItem("TRIANGLE_SLOPE");
    connect(slopeMetricComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(slopeMetricComboBoxIndexChanged(int)));
    travFormLayout->addRow("Slope Metric Type:", slopeMetricComboBox);

    obstacleDistanceSpinBox = new QDoubleSpinBox();
    obstacleDistanceSpinBox->setMaximum(99999);
    obstacleDistanceSpinBox->setMinimum(0);
    obstacleDistanceSpinBox->setDecimals(2);
    connect(obstacleDistanceSpinBox, SIGNAL(editingFinished()), this, SLOT(obstacleDistanceSpinBoxEditingFinished()));
    travFormLayout->addRow("Obstacle Distance:", obstacleDistanceSpinBox);

    travGridResolutionSpinBox = new QDoubleSpinBox();
    travGridResolutionSpinBox->setMinimum(0.05);
    travGridResolutionSpinBox->setMaximum(5.0);
    travGridResolutionSpinBox->setSingleStep(0.05);
    travGridResolutionSpinBox->setDecimals(2);
    connect(travGridResolutionSpinBox, SIGNAL(editingFinished()), this, SLOT(travGridResolutionEditingFinished()));
    travFormLayout->addRow("Grid Resolution (m):", travGridResolutionSpinBox);

    travMaxStepHeightSpinBox = new QDoubleSpinBox();
    travMaxStepHeightSpinBox->setMinimum(0.0);
    travMaxStepHeightSpinBox->setMaximum(5.0);
    travMaxStepHeightSpinBox->setSingleStep(0.05);
    travMaxStepHeightSpinBox->setDecimals(2);
    connect(travMaxStepHeightSpinBox, SIGNAL(editingFinished()), this, SLOT(travMaxStepHeightEditingFinished()));
    travFormLayout->addRow("Max Step Height (m):", travMaxStepHeightSpinBox);

    travMinTraversablePercentageSpinBox = new QDoubleSpinBox();
    travMinTraversablePercentageSpinBox->setMinimum(0.0);
    travMinTraversablePercentageSpinBox->setMaximum(1.0);
    travMinTraversablePercentageSpinBox->setSingleStep(0.05);
    travMinTraversablePercentageSpinBox->setDecimals(2);
    connect(travMinTraversablePercentageSpinBox, SIGNAL(editingFinished()), this, SLOT(travMinTraversablePercentageEditingFinished()));
    travFormLayout->addRow("Min Traversable Percentage:", travMinTraversablePercentageSpinBox);

    travObstacleInflationMultiplierSpinBox = new QDoubleSpinBox();
    travObstacleInflationMultiplierSpinBox->setMinimum(0.0);
    travObstacleInflationMultiplierSpinBox->setMaximum(10.0);
    travObstacleInflationMultiplierSpinBox->setSingleStep(0.1);
    travObstacleInflationMultiplierSpinBox->setDecimals(2);
    connect(travObstacleInflationMultiplierSpinBox, SIGNAL(editingFinished()), this, SLOT(travObstacleInflationMultiplierEditingFinished()));
    travFormLayout->addRow("Obstacle Inflation Multiplier:", travObstacleInflationMultiplierSpinBox);



    travAllowForwardDownhillCheckBox = new QCheckBox();
    connect(travAllowForwardDownhillCheckBox, SIGNAL(stateChanged(int)), this, SLOT(travAllowForwardDownhillStateChanged(int)));
    travFormLayout->addRow("Allow Forward Downhill", travAllowForwardDownhillCheckBox);

    travEnableInclineLimittingCheckBox = new QCheckBox();
    connect(travEnableInclineLimittingCheckBox, SIGNAL(stateChanged(int)), this, SLOT(travEnableInclineLimittingStateChanged(int)));
    travFormLayout->addRow("Enable Incline Limiting", travEnableInclineLimittingCheckBox);

    obstacleFactorSpinBox = nullptr; // Obstacle factor functionality was removed in base code

    travTab->setLayout(travFormLayout);
    tabWidget->addTab(travTab, "Traversability/Terrain");

    // Tab 3: Motion Primitives (Splines)
    QWidget* splineTab = new QWidget();
    QFormLayout* splineFormLayout = new QFormLayout();

    splineGridSizeSpinBox = new QDoubleSpinBox();
    splineGridSizeSpinBox->setMinimum(0.05);
    splineGridSizeSpinBox->setMaximum(5.0);
    splineGridSizeSpinBox->setSingleStep(0.05);
    splineGridSizeSpinBox->setDecimals(2);
    connect(splineGridSizeSpinBox, SIGNAL(editingFinished()), this, SLOT(splineGridSizeEditingFinished()));
    splineFormLayout->addRow("Spline Grid Size (m):", splineGridSizeSpinBox);

    splineDestCircleRadiusSpinBox = new QDoubleSpinBox();
    splineDestCircleRadiusSpinBox->setMinimum(1.0);
    splineDestCircleRadiusSpinBox->setMaximum(1000.0);
    splineDestCircleRadiusSpinBox->setSingleStep(1.0);
    splineDestCircleRadiusSpinBox->setDecimals(1);
    connect(splineDestCircleRadiusSpinBox, SIGNAL(editingFinished()), this, SLOT(splineDestCircleRadiusEditingFinished()));
    splineFormLayout->addRow("Destination Circle Radius:", splineDestCircleRadiusSpinBox);

    splineCellSkipFactorSpinBox = new QDoubleSpinBox();
    splineCellSkipFactorSpinBox->setMinimum(0.001);
    splineCellSkipFactorSpinBox->setMaximum(1000.0);
    splineCellSkipFactorSpinBox->setSingleStep(0.01);
    splineCellSkipFactorSpinBox->setDecimals(3);
    connect(splineCellSkipFactorSpinBox, SIGNAL(editingFinished()), this, SLOT(splineCellSkipFactorEditingFinished()));
    splineFormLayout->addRow("Cell Skip Factor:", splineCellSkipFactorSpinBox);

    splineNumAnglesSpinBox = new QSpinBox();
    splineNumAnglesSpinBox->setMinimum(1);
    splineNumAnglesSpinBox->setMaximum(128);
    connect(splineNumAnglesSpinBox, SIGNAL(valueChanged(int)), this, SLOT(splineNumAnglesValueChanged(int)));
    splineFormLayout->addRow("Num Angles:", splineNumAnglesSpinBox);

    splineNumEndAnglesSpinBox = new QSpinBox();
    splineNumEndAnglesSpinBox->setMinimum(1);
    splineNumEndAnglesSpinBox->setMaximum(128);
    connect(splineNumEndAnglesSpinBox, SIGNAL(valueChanged(int)), this, SLOT(splineNumEndAnglesValueChanged(int)));
    splineFormLayout->addRow("Num End Angles:", splineNumEndAnglesSpinBox);

    splineOrderSpinBox = new QSpinBox();
    splineOrderSpinBox->setMinimum(1);
    splineOrderSpinBox->setMaximum(10);
    connect(splineOrderSpinBox, SIGNAL(valueChanged(int)), this, SLOT(splineOrderValueChanged(int)));
    splineFormLayout->addRow("Spline Order:", splineOrderSpinBox);

    splineGenPointTurnMotionsCheckBox = new QCheckBox();
    connect(splineGenPointTurnMotionsCheckBox, SIGNAL(stateChanged(int)), this, SLOT(splineGenPointTurnMotionsStateChanged(int)));
    splineFormLayout->addRow("Generate Point Turn Motions", splineGenPointTurnMotionsCheckBox);

    splineGenLateralMotionsCheckBox = new QCheckBox();
    connect(splineGenLateralMotionsCheckBox, SIGNAL(stateChanged(int)), this, SLOT(splineGenLateralMotionsStateChanged(int)));
    splineFormLayout->addRow("Generate Lateral Motions", splineGenLateralMotionsCheckBox);

    splineGenBackwardMotionsCheckBox = new QCheckBox();
    connect(splineGenBackwardMotionsCheckBox, SIGNAL(stateChanged(int)), this, SLOT(splineGenBackwardMotionsStateChanged(int)));
    splineFormLayout->addRow("Generate Backward Motions", splineGenBackwardMotionsCheckBox);

    splineGenForwardMotionsCheckBox = new QCheckBox();
    connect(splineGenForwardMotionsCheckBox, SIGNAL(stateChanged(int)), this, SLOT(splineGenForwardMotionsStateChanged(int)));
    splineFormLayout->addRow("Generate Forward Motions", splineGenForwardMotionsCheckBox);

    splineTab->setLayout(splineFormLayout);
    tabWidget->addTab(splineTab, "Motion Primitives");

    // Tab 4: Planner/Search
    QWidget* planTab = new QWidget();
    QFormLayout* planFormLayout = new QFormLayout();

    time = new QDoubleSpinBox();
    time->setMinimum(1);
    time->setMaximum(9999999);
    time->setDecimals(2);
    connect(time, SIGNAL(editingFinished()), this, SLOT(timeEditingFinished()));
    planFormLayout->addRow("Max processor time (seconds):", time);

    numThreadsSpinBox = new QSpinBox();
    numThreadsSpinBox->setMinimum(1);
    numThreadsSpinBox->setMaximum(128);
    connect(numThreadsSpinBox, SIGNAL(valueChanged(int)), this, SLOT(numThreadsValueChanged(int)));
    planFormLayout->addRow("Number of Threads:", numThreadsSpinBox);

    planEpsilonStepsSpinBox = new QDoubleSpinBox();
    planEpsilonStepsSpinBox->setMinimum(0.01);
    planEpsilonStepsSpinBox->setMaximum(100.0);
    planEpsilonStepsSpinBox->setSingleStep(0.5);
    planEpsilonStepsSpinBox->setDecimals(2);
    connect(planEpsilonStepsSpinBox, SIGNAL(editingFinished()), this, SLOT(planEpsilonStepsEditingFinished()));
    planFormLayout->addRow("Epsilon Steps:", planEpsilonStepsSpinBox);

    planInitialEpsilonSpinBox = new QDoubleSpinBox();
    planInitialEpsilonSpinBox->setMinimum(1.0);
    planInitialEpsilonSpinBox->setMaximum(1000.0);
    planInitialEpsilonSpinBox->setSingleStep(0.5);
    planInitialEpsilonSpinBox->setDecimals(2);
    connect(planInitialEpsilonSpinBox, SIGNAL(editingFinished()), this, SLOT(planInitialEpsilonEditingFinished()));
    planFormLayout->addRow("Initial Epsilon:", planInitialEpsilonSpinBox);

    mobMultForwardSpinBox = new QSpinBox();
    mobMultForwardSpinBox->setMinimum(1);
    mobMultForwardSpinBox->setMaximum(100);
    connect(mobMultForwardSpinBox, SIGNAL(valueChanged(int)), this, SLOT(mobMultForwardValueChanged(int)));
    planFormLayout->addRow("Cost Mult: Forward:", mobMultForwardSpinBox);

    mobMultBackwardSpinBox = new QSpinBox();
    mobMultBackwardSpinBox->setMinimum(1);
    mobMultBackwardSpinBox->setMaximum(100);
    connect(mobMultBackwardSpinBox, SIGNAL(valueChanged(int)), this, SLOT(mobMultBackwardValueChanged(int)));
    planFormLayout->addRow("Cost Mult: Backward:", mobMultBackwardSpinBox);

    mobMultLateralSpinBox = new QSpinBox();
    mobMultLateralSpinBox->setMinimum(1);
    mobMultLateralSpinBox->setMaximum(100);
    connect(mobMultLateralSpinBox, SIGNAL(valueChanged(int)), this, SLOT(mobMultLateralValueChanged(int)));
    planFormLayout->addRow("Cost Mult: Lateral:", mobMultLateralSpinBox);

    mobMultForwardTurnSpinBox = new QSpinBox();
    mobMultForwardTurnSpinBox->setMinimum(1);
    mobMultForwardTurnSpinBox->setMaximum(100);
    connect(mobMultForwardTurnSpinBox, SIGNAL(valueChanged(int)), this, SLOT(mobMultForwardTurnValueChanged(int)));
    planFormLayout->addRow("Cost Mult: Forward Turn:", mobMultForwardTurnSpinBox);

    mobMultBackwardTurnSpinBox = new QSpinBox();
    mobMultBackwardTurnSpinBox->setMinimum(1);
    mobMultBackwardTurnSpinBox->setMaximum(100);
    connect(mobMultBackwardTurnSpinBox, SIGNAL(valueChanged(int)), this, SLOT(mobMultBackwardTurnValueChanged(int)));
    planFormLayout->addRow("Cost Mult: Backward Turn:", mobMultBackwardTurnSpinBox);

    mobMultPointTurnSpinBox = new QSpinBox();
    mobMultPointTurnSpinBox->setMinimum(1);
    mobMultPointTurnSpinBox->setMaximum(100);
    connect(mobMultPointTurnSpinBox, SIGNAL(valueChanged(int)), this, SLOT(mobMultPointTurnValueChanged(int)));
    planFormLayout->addRow("Cost Mult: Point Turn:", mobMultPointTurnSpinBox);

    mobMultLateralCurveSpinBox = new QSpinBox();
    mobMultLateralCurveSpinBox->setMinimum(1);
    mobMultLateralCurveSpinBox->setMaximum(100);
    connect(mobMultLateralCurveSpinBox, SIGNAL(valueChanged(int)), this, SLOT(mobMultLateralCurveValueChanged(int)));
    planFormLayout->addRow("Cost Mult: Lateral Curve:", mobMultLateralCurveSpinBox);

    planUsePathStatisticsCheckBox = new QCheckBox();
    connect(planUsePathStatisticsCheckBox, SIGNAL(stateChanged(int)), this, SLOT(planUsePathStatisticsStateChanged(int)));
    planFormLayout->addRow("Use Path Statistics", planUsePathStatisticsCheckBox);

    planSearchUntilFirstSolutionCheckBox = new QCheckBox();
    connect(planSearchUntilFirstSolutionCheckBox, SIGNAL(stateChanged(int)), this, SLOT(planSearchUntilFirstSolutionStateChanged(int)));
    planFormLayout->addRow("Search Until First Solution", planSearchUntilFirstSolutionCheckBox);

    planTab->setLayout(planFormLayout);
    tabWidget->addTab(planTab, "Planner/Search");

    layout->addWidget(tabWidget);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    QPushButton* replanButton = new QPushButton("Plan");
    QPushButton* updateParamsButton = new QPushButton("Update Parameters");
    QPushButton* dumpButton = new QPushButton("Create PlannerDump");
    buttonLayout->addWidget(replanButton);
    buttonLayout->addWidget(updateParamsButton);
    buttonLayout->addWidget(dumpButton);
    
    layout->addLayout(buttonLayout);

    connect(replanButton, SIGNAL(released()), this, SLOT(replanButtonReleased()));
    connect(updateParamsButton, SIGNAL(released()), this, SLOT(updateParamsButtonReleased()));
    connect(dumpButton, SIGNAL(released()), this, SLOT(dumpPressed()));

    bar = new QProgressBar();
    bar->setMinimum(0);
    bar->setMaximum(1);
    layout->addWidget(bar);
    
    window.setLayout(layout);

    //to be able to send Trajectory via slot
    qRegisterMetaType<std::vector<ugv_nav4d::Motion>>("std::vector<ugv_nav4d::Motion>");
    qRegisterMetaType<std::vector<base::Trajectory>>("std::vector<base::Trajectory>");
    qRegisterMetaType<maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase*>>("maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase*>");

    connect(&mlsViz, SIGNAL(picked(float,float,float, int, int)), this, SLOT(picked(float,float,float, int, int)));
    connect(&trav3dViz, SIGNAL(picked(float,float,float, int, int)), this, SLOT(picked(float,float,float, int, int)));
    connect(this, SIGNAL(plannerDone()), this, SLOT(plannerIsDone()));
}


void PlannerGui::setupPlanner(int argc, char** argv)
{
    // Load config from default path or fall back to gui/config/parameters.yaml
    boost::filesystem::path configPath("/home/dfki.uni-bremen.de/mlodhi/ROCK/Docker/docker_arter_ros2_jazzy/workspace/src/launch/arter_bringup/config/yaml/ugv_nav4d_params.yaml");
    if (!boost::filesystem::exists(configPath)) {
        configPath = boost::filesystem::path(__FILE__).parent_path() / "config" / "parameters.yaml";
    }
    
    if(boost::filesystem::exists(configPath)) {
        LOG_INFO_S << "Loading configuration from: " << configPath.string();
        if(!ConfigLoader::loadConfig(configPath.string(), splineConfig, mobilityConfig, travConfig, plannerConfig)) {
            LOG_WARN_S << "Failed to load config, using defaults";
            setupDefaultConfigs();
        }
    } else {
        LOG_WARN_S << "Config file not found at: " << configPath.string() << ", using defaults";
        setupDefaultConfigs();
    }

    planner.reset(new ugv_nav4d::Planner(splineConfig, travConfig, mobilityConfig, plannerConfig));
    travGen.reset(new traversability_generator3d::TraversabilityGenerator3d(travConfig));

    sbpl_spline_primitives::SbplSplineMotionPrimitives primitives(splineConfig);
    splineViz.setMaxCurvature(ugv_nav4d::PreComputedMotions::calculateCurvatureFromRadius(mobilityConfig.minTurningRadius));
    splineViz.updateData(primitives);

    if(argc > 1)
    {
        const std::string mls(argv[1]);
        loadMls(mls);
    }
    else
    {
        loadMls();
    }
    updateWidgetValues();
}

void PlannerGui::setupDefaultConfigs()
{
    splineConfig.gridSize = 0.5;
    splineConfig.numAngles = 42;
    splineConfig.numEndAngles = 21;
    splineConfig.destinationCircleRadius = 10;
    splineConfig.cellSkipFactor = 0.1;
    splineConfig.generatePointTurnMotions = false;
    splineConfig.generateLateralMotions = false;
    splineConfig.generateBackwardMotions = true;
    splineConfig.generateForwardMotions = true;
    splineConfig.splineOrder = 4;

    mobilityConfig.translationSpeed = 1.0;
    mobilityConfig.rotationSpeed = 1.0;
    mobilityConfig.minTurningRadius = 5.0;
    mobilityConfig.searchRadius = 1.0;
    mobilityConfig.searchProgressSteps = 0.1;
    mobilityConfig.multiplierForward = 1.0;
    mobilityConfig.multiplierForwardTurn = 1.2;
    mobilityConfig.multiplierBackward = 2.4;
    mobilityConfig.multiplierBackwardTurn = 3.0;
    mobilityConfig.multiplierLateral = 4.0;
    mobilityConfig.multiplierLateralCurve = 4.0;
    mobilityConfig.multiplierPointTurn = 2.0;
    mobilityConfig.maxMotionCurveLength = 100.0;
    mobilityConfig.spline_sampling_resolution = 0.1;
    mobilityConfig.remove_goal_offset = false;
    mobilityConfig.curvaturePenaltyWeight = 0.0;
    mobilityConfig.angularCostWeight = 1.0;

    travConfig.gridResolution = 0.5;
    travConfig.maxSlope = 1.0;
    travConfig.maxStepHeight = 1.0;
    travConfig.robotSizeX = 6.0;
    travConfig.robotSizeY = 3.5;
    travConfig.robotHeight = 3.0;
    travConfig.slopeMetricScale = 1.0;
    travConfig.slopeMetric = traversability_generator3d::SlopeMetric::NONE;
    travConfig.inclineLimittingMinSlope = 0.2;
    travConfig.inclineLimittingLimit = 0.1;
    travConfig.costFunctionDist = 0.0;
    travConfig.distToGround = 0.0;
    travConfig.minTraversablePercentage = 0.4;
    travConfig.enableInclineLimitting = false;
    travConfig.obstacleInflationMultiplier = 0.4;


    plannerConfig.epsilonSteps = 2.0;
    plannerConfig.initialEpsilon = 64.0;
    plannerConfig.numThreads = 8;
}


void PlannerGui::loadMls()
{
    const QString file = QFileDialog::getOpenFileName(nullptr, tr("Load mls map"),
                                                        QDir::currentPath(), QString(),
                                                        nullptr, QFileDialog::DontUseNativeDialog);
    if(!file.isEmpty())
    {
        loadMls(file.toStdString());
    }
}

void PlannerGui::loadMls(const std::string& path)
{
    std::ifstream fileIn(path);       
    

    if(path.find(".ply") != std::string::npos)
    {
        LOG_INFO_S << "Loading PLY";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PLYReader plyReader;
        if(plyReader.read(path, *cloud) >= 0)
        {
            pcl::PointXYZ mi, ma; 
            pcl::getMinMax3D (*cloud, mi, ma); 
            LOG_INFO_S << "MIN: " << mi << ", MAX: " << ma;

            const double mls_res = travConfig.gridResolution;
            const double size_x = ma.x - mi.x;
            const double size_y = ma.y - mi.y;

            const maps::grid::Vector2ui numCells(size_x / mls_res + 2, size_y / mls_res + 2);
            LOG_INFO_S << "NUM CELLS: " << numCells.transpose();

            maps::grid::MLSConfig cfg;
            cfg.gapSize = 0.1;
            const maps::grid::Vector2d mapSize(numCells[0]*mls_res, numCells[1]*mls_res);
            const maps::grid::Vector3d offset(mi.x-0.5*mls_res, mi.y-0.5*mls_res, 0);
            LOG_DEBUG_S << "Range(x): [" << offset[0] << "; " << mapSize[0]+offset[0] << "], "
            << "Range(y): [" << offset[1] << "; " << mapSize[1]+offset[1] << "]\n";

            mlsMap = maps::grid::MLSMapSloped(numCells, maps::grid::Vector2d(mls_res, mls_res), cfg);
            mlsMap.translate(offset);
            mlsMap.mergePointCloud(*cloud, base::Transform3d::Identity());
            mlsViz.updateMLSSloped(mlsMap);

            std::shared_ptr<maps::grid::MLSMapSloped> mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mlsMap);
            travGen->setMLSGrid(mlsPtr);            
        }
        return;
    }
    try
    {
        LOG_INFO_S << "Loading MLS";
        boost::archive::binary_iarchive mlsIn(fileIn);
        mlsIn >> mlsMap;
        mlsViz.updateMLSSloped(mlsMap);

        std::shared_ptr<maps::grid::MLSMapSloped> mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mlsMap);
        travGen->setMLSGrid(mlsPtr);    
        return;
    }
    catch(...) {}
    

    std::cerr << "Unabled to load mls. Unknown format";
    
}


void PlannerGui::picked(float x, float y, float z, int buttonMask, int modifierMask)
{   
//     start << 5.91327,  1.38306, -1.39575;
//     goal <<  7.47328,  1.34183, -1.39437;
//     startPlanThread();
    
    //1 = left click
    //4 = right click
    
    switch(buttonMask)
    {
        case 1: //left click
        {
            start.position << x, y, z;
            start.position.z() += travConfig.distToGround; //because we click on the ground but need to put robot position

#ifdef ENABLE_DEBUG_DRAWINGS
            V3DD::CLEAR_DRAWING("ugv_nav4d_start_aabb");
            V3DD::DRAW_WIREFRAME_BOX("ugv_nav4d_start_aabb", start.position +  base::Vector3d(0, 0, travConfig.distToGround / 2.0), start.orientation,
                               base::Vector3d(travConfig.robotSizeX, travConfig.robotSizeY, travConfig.robotHeight - travConfig.distToGround), V3DD::Color::cyan);
#endif
            QVector3D pos(start.position.x(), start.position.y(), start.position.z());
            startViz.setTranslation(pos);
            LOG_INFO_S << "Start: " << start.position.transpose();
            startPicked = true;
        }
            break;
        case 4: //right click
        {
            goal.position << x, y, z;
            goal.position.z() += travConfig.distToGround;
            QVector3D pos(goal.position.x(), goal.position.y(), goal.position.z());
            goalViz.setTranslation(pos);
            LOG_INFO_S << "goal: " << goal.position.transpose();
            goalPicked = true;
        }
            break;
        default:
            break;
    }
}

void PlannerGui::show()
{
    window.show();
}

void PlannerGui::maxSlopeEditingFinished()
{
    travConfig.maxSlope = maxSlopeSpinBox->value()/180.0 * M_PI;
}

void PlannerGui::inclineLimittingLimitSpinBoxEditingFinished()
{
    travConfig.inclineLimittingLimit = inclineLimittingLimitSpinBox->value()/180.0 * M_PI;
}

void PlannerGui::inclineLimittingMinSlopeSpinBoxEditingFinished()
{
    travConfig.inclineLimittingMinSlope = inclineLimittingMinSlopeSpinBox->value()/180.0 * M_PI;
}

void PlannerGui::slopeMetricScaleSpinBoxEditingFinished()
{
    travConfig.slopeMetricScale = slopeMetricScaleSpinBox->value();
}

void PlannerGui::slopeMetricComboBoxIndexChanged(int index)
{
    std::vector<traversability_generator3d::SlopeMetric> metrics = {traversability_generator3d::SlopeMetric::NONE, 
                                        traversability_generator3d::SlopeMetric::AVG_SLOPE, 
                                        traversability_generator3d::SlopeMetric::MAX_SLOPE,
                                        traversability_generator3d::SlopeMetric::TRIANGLE_SLOPE};
    if(size_t(index) < metrics.size())
    {
        travConfig.slopeMetric = metrics[index];
    }
    else
    {
        throw std::runtime_error("unknown slope index");
    }
}

void PlannerGui::numThreadsValueChanged(int newValue)
{
    if(newValue >= 1)
    {
        plannerConfig.numThreads = newValue; 
    }
}


void PlannerGui::goalOrientationChanged(int newValue)
{
    const double rad = newValue/180.0 * M_PI;
    goal.orientation = Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitZ());
    goalViz.setRotation(QQuaternion(goal.orientation.w(), goal.orientation.x(), goal.orientation.y(), goal.orientation.z()));
}

void PlannerGui::startOrientationChanged(int newValue)
{
    const double rad = newValue/180.0 * M_PI;
    start.orientation = Eigen::AngleAxisd(rad, Eigen::Vector3d::UnitZ());
    startViz.setRotation(QQuaternion(start.orientation.w(), start.orientation.x(), start.orientation.y(), start.orientation.z()));
#ifdef ENABLE_DEBUG_DRAWINGS
    V3DD::CLEAR_DRAWING("ugv_nav4d_start_aabb");
    V3DD::DRAW_WIREFRAME_BOX("ugv_nav4d_start_aabb", start.position + Eigen::Vector3d(0, 0, travConfig.distToGround),
                       start.orientation, base::Vector3d(travConfig.robotSizeX, travConfig.robotSizeY, travConfig.robotHeight), V3DD::Color::cyan);
#endif
}

void PlannerGui::obstacleDistanceSpinBoxEditingFinished()
{
    travConfig.costFunctionDist = obstacleDistanceSpinBox->value();
}

void PlannerGui::obstacleFactorSpinBoxEditingFinished()
{
    throw std::runtime_error("Function removed");
//     travConfig.costFunctionObstacleMultiplier = obstacleFactorSpinBox->value();
}

void PlannerGui::timeEditingFinished()
{
    
}

void PlannerGui::robotSizeXEditingFinished()
{
    travConfig.robotSizeX = robotSizeXSpinBox->value();
}

void PlannerGui::robotSizeYEditingFinished()
{
    travConfig.robotSizeY = robotSizeYSpinBox->value();
}

void PlannerGui::robotHeightEditingFinished()
{
    travConfig.robotHeight = robotHeightSpinBox->value();
}

void PlannerGui::distToGroundEditingFinished()
{
    travConfig.distToGround = distToGroundSpinBox->value();
}

void PlannerGui::translationSpeedEditingFinished()
{
    mobilityConfig.translationSpeed = translationSpeedSpinBox->value();
}

void PlannerGui::rotationSpeedEditingFinished()
{
    mobilityConfig.rotationSpeed = rotationSpeedSpinBox->value();
}

void PlannerGui::minTurningRadiusEditingFinished()
{
    mobilityConfig.minTurningRadius = minTurningRadiusSpinBox->value();
    splineViz.setMaxCurvature(ugv_nav4d::PreComputedMotions::calculateCurvatureFromRadius(mobilityConfig.minTurningRadius));
    sbpl_spline_primitives::SbplSplineMotionPrimitives primitives(splineConfig);
    splineViz.updateData(primitives);
}

void PlannerGui::updateWidgetValues()
{
    const bool wasBlockedMaxSlope = maxSlopeSpinBox->blockSignals(true);
    maxSlopeSpinBox->setValue(travConfig.maxSlope * 180.0 / M_PI);
    maxSlopeSpinBox->blockSignals(wasBlockedMaxSlope);

    const bool wasBlockedMinSlope = inclineLimittingMinSlopeSpinBox->blockSignals(true);
    inclineLimittingMinSlopeSpinBox->setValue(travConfig.inclineLimittingMinSlope * 180.0 / M_PI);
    inclineLimittingMinSlopeSpinBox->blockSignals(wasBlockedMinSlope);

    const bool wasBlockedLimit = inclineLimittingLimitSpinBox->blockSignals(true);
    inclineLimittingLimitSpinBox->setValue(travConfig.inclineLimittingLimit * 180.0 / M_PI);
    inclineLimittingLimitSpinBox->blockSignals(wasBlockedLimit);

    const bool wasBlockedScale = slopeMetricScaleSpinBox->blockSignals(true);
    slopeMetricScaleSpinBox->setValue(travConfig.slopeMetricScale);
    slopeMetricScaleSpinBox->blockSignals(wasBlockedScale);

    const bool wasBlockedMetric = slopeMetricComboBox->blockSignals(true);
    int metricIndex = 0;
    switch(travConfig.slopeMetric)
    {
        case traversability_generator3d::SlopeMetric::NONE:
            metricIndex = 0;
            break;
        case traversability_generator3d::SlopeMetric::AVG_SLOPE:
            metricIndex = 1;
            break;
        case traversability_generator3d::SlopeMetric::MAX_SLOPE:
            metricIndex = 2;
            break;
        case traversability_generator3d::SlopeMetric::TRIANGLE_SLOPE:
            metricIndex = 3;
            break;
    }
    slopeMetricComboBox->setCurrentIndex(metricIndex);
    slopeMetricComboBox->blockSignals(wasBlockedMetric);

    const bool wasBlockedObstacleDist = obstacleDistanceSpinBox->blockSignals(true);
    obstacleDistanceSpinBox->setValue(travConfig.costFunctionDist);
    obstacleDistanceSpinBox->blockSignals(wasBlockedObstacleDist);

    const bool wasBlockedNumThreads = numThreadsSpinBox->blockSignals(true);
    numThreadsSpinBox->setValue(plannerConfig.numThreads);
    numThreadsSpinBox->blockSignals(wasBlockedNumThreads);

    const bool wasBlockedSizeX = robotSizeXSpinBox->blockSignals(true);
    robotSizeXSpinBox->setValue(travConfig.robotSizeX);
    robotSizeXSpinBox->blockSignals(wasBlockedSizeX);

    const bool wasBlockedSizeY = robotSizeYSpinBox->blockSignals(true);
    robotSizeYSpinBox->setValue(travConfig.robotSizeY);
    robotSizeYSpinBox->blockSignals(wasBlockedSizeY);

    const bool wasBlockedHeight = robotHeightSpinBox->blockSignals(true);
    robotHeightSpinBox->setValue(travConfig.robotHeight);
    robotHeightSpinBox->blockSignals(wasBlockedHeight);

    const bool wasBlockedGround = distToGroundSpinBox->blockSignals(true);
    distToGroundSpinBox->setValue(travConfig.distToGround);
    distToGroundSpinBox->blockSignals(wasBlockedGround);

    const bool wasBlockedTrans = translationSpeedSpinBox->blockSignals(true);
    translationSpeedSpinBox->setValue(mobilityConfig.translationSpeed);
    translationSpeedSpinBox->blockSignals(wasBlockedTrans);

    const bool wasBlockedRot = rotationSpeedSpinBox->blockSignals(true);
    rotationSpeedSpinBox->setValue(mobilityConfig.rotationSpeed);
    rotationSpeedSpinBox->blockSignals(wasBlockedRot);

    const bool wasBlockedRadius = minTurningRadiusSpinBox->blockSignals(true);
    minTurningRadiusSpinBox->setValue(mobilityConfig.minTurningRadius);
    minTurningRadiusSpinBox->blockSignals(wasBlockedRadius);

    const bool wasBlockedStartSlider = startOrientatationSlider->blockSignals(true);
    startOrientatationSlider->setValue(int(base::getYaw(start.orientation) * 180.0 / M_PI + 0.5));
    startOrientatationSlider->blockSignals(wasBlockedStartSlider);

    const bool wasBlockedGoalSlider = goalOrientationSlider->blockSignals(true);
    goalOrientationSlider->setValue(int(base::getYaw(goal.orientation) * 180.0 / M_PI + 0.5));
    goalOrientationSlider->blockSignals(wasBlockedGoalSlider);

    // Spline Config Widget Updates
    const bool wasBlockedSplineGridSize = splineGridSizeSpinBox->blockSignals(true);
    splineGridSizeSpinBox->setValue(splineConfig.gridSize);
    splineGridSizeSpinBox->blockSignals(wasBlockedSplineGridSize);

    const bool wasBlockedSplineDestRadius = splineDestCircleRadiusSpinBox->blockSignals(true);
    splineDestCircleRadiusSpinBox->setValue(splineConfig.destinationCircleRadius);
    splineDestCircleRadiusSpinBox->blockSignals(wasBlockedSplineDestRadius);

    const bool wasBlockedSplineCellSkip = splineCellSkipFactorSpinBox->blockSignals(true);
    splineCellSkipFactorSpinBox->setValue(splineConfig.cellSkipFactor);
    splineCellSkipFactorSpinBox->blockSignals(wasBlockedSplineCellSkip);

    const bool wasBlockedSplineNumAngles = splineNumAnglesSpinBox->blockSignals(true);
    splineNumAnglesSpinBox->setValue(splineConfig.numAngles);
    splineNumAnglesSpinBox->blockSignals(wasBlockedSplineNumAngles);

    const bool wasBlockedSplineNumEndAngles = splineNumEndAnglesSpinBox->blockSignals(true);
    splineNumEndAnglesSpinBox->setValue(splineConfig.numEndAngles);
    splineNumEndAnglesSpinBox->blockSignals(wasBlockedSplineNumEndAngles);

    const bool wasBlockedSplineOrder = splineOrderSpinBox->blockSignals(true);
    splineOrderSpinBox->setValue(splineConfig.splineOrder);
    splineOrderSpinBox->blockSignals(wasBlockedSplineOrder);

    const bool wasBlockedSplinePointTurn = splineGenPointTurnMotionsCheckBox->blockSignals(true);
    splineGenPointTurnMotionsCheckBox->setChecked(splineConfig.generatePointTurnMotions);
    splineGenPointTurnMotionsCheckBox->blockSignals(wasBlockedSplinePointTurn);

    const bool wasBlockedSplineLateral = splineGenLateralMotionsCheckBox->blockSignals(true);
    splineGenLateralMotionsCheckBox->setChecked(splineConfig.generateLateralMotions);
    splineGenLateralMotionsCheckBox->blockSignals(wasBlockedSplineLateral);

    const bool wasBlockedSplineBackward = splineGenBackwardMotionsCheckBox->blockSignals(true);
    splineGenBackwardMotionsCheckBox->setChecked(splineConfig.generateBackwardMotions);
    splineGenBackwardMotionsCheckBox->blockSignals(wasBlockedSplineBackward);

    const bool wasBlockedSplineForward = splineGenForwardMotionsCheckBox->blockSignals(true);
    splineGenForwardMotionsCheckBox->setChecked(splineConfig.generateForwardMotions);
    splineGenForwardMotionsCheckBox->blockSignals(wasBlockedSplineForward);

    // Mobility Config Widget Updates
    const bool wasBlockedMobSearchRadius = mobSearchRadiusSpinBox->blockSignals(true);
    mobSearchRadiusSpinBox->setValue(mobilityConfig.searchRadius);
    mobSearchRadiusSpinBox->blockSignals(wasBlockedMobSearchRadius);

    const bool wasBlockedMobSearchProgress = mobSearchProgressStepsSpinBox->blockSignals(true);
    mobSearchProgressStepsSpinBox->setValue(mobilityConfig.searchProgressSteps);
    mobSearchProgressStepsSpinBox->blockSignals(wasBlockedMobSearchProgress);

    const bool wasBlockedMobMaxCurve = mobMaxMotionCurveLengthSpinBox->blockSignals(true);
    mobMaxMotionCurveLengthSpinBox->setValue(mobilityConfig.maxMotionCurveLength);
    mobMaxMotionCurveLengthSpinBox->blockSignals(wasBlockedMobMaxCurve);

    const bool wasBlockedMobSampling = mobSplineSamplingResSpinBox->blockSignals(true);
    mobSplineSamplingResSpinBox->setValue(mobilityConfig.spline_sampling_resolution);
    mobSplineSamplingResSpinBox->blockSignals(wasBlockedMobSampling);

    const bool wasBlockedMobRemoveOffset = mobRemoveGoalOffsetCheckBox->blockSignals(true);
    mobRemoveGoalOffsetCheckBox->setChecked(mobilityConfig.remove_goal_offset);
    mobRemoveGoalOffsetCheckBox->blockSignals(wasBlockedMobRemoveOffset);

    const bool wasBlockedMobCurvature = mobCurvaturePenaltyWeightSpinBox->blockSignals(true);
    mobCurvaturePenaltyWeightSpinBox->setValue(mobilityConfig.curvaturePenaltyWeight);
    mobCurvaturePenaltyWeightSpinBox->blockSignals(wasBlockedMobCurvature);

    const bool wasBlockedMobAngular = mobAngularCostWeightSpinBox->blockSignals(true);
    mobAngularCostWeightSpinBox->setValue(mobilityConfig.angularCostWeight);
    mobAngularCostWeightSpinBox->blockSignals(wasBlockedMobAngular);

    // Traversability Config Widget Updates
    const bool wasBlockedTravGridRes = travGridResolutionSpinBox->blockSignals(true);
    travGridResolutionSpinBox->setValue(travConfig.gridResolution);
    travGridResolutionSpinBox->blockSignals(wasBlockedTravGridRes);

    const bool wasBlockedTravStep = travMaxStepHeightSpinBox->blockSignals(true);
    travMaxStepHeightSpinBox->setValue(travConfig.maxStepHeight);
    travMaxStepHeightSpinBox->blockSignals(wasBlockedTravStep);

    const bool wasBlockedTravMinPercentage = travMinTraversablePercentageSpinBox->blockSignals(true);
    travMinTraversablePercentageSpinBox->setValue(travConfig.minTraversablePercentage);
    travMinTraversablePercentageSpinBox->blockSignals(wasBlockedTravMinPercentage);

    const bool wasBlockedTravInflation = travObstacleInflationMultiplierSpinBox->blockSignals(true);
    travObstacleInflationMultiplierSpinBox->setValue(travConfig.obstacleInflationMultiplier);
    travObstacleInflationMultiplierSpinBox->blockSignals(wasBlockedTravInflation);



    const bool wasBlockedTravAllowDownhill = travAllowForwardDownhillCheckBox->blockSignals(true);
    travAllowForwardDownhillCheckBox->setChecked(travConfig.allowForwardDownhill);
    travAllowForwardDownhillCheckBox->blockSignals(wasBlockedTravAllowDownhill);

    const bool wasBlockedTravEnableIncline = travEnableInclineLimittingCheckBox->blockSignals(true);
    travEnableInclineLimittingCheckBox->setChecked(travConfig.enableInclineLimitting);
    travEnableInclineLimittingCheckBox->blockSignals(wasBlockedTravEnableIncline);

    // Planner Config Widget Updates
    const bool wasBlockedPlanEpsSteps = planEpsilonStepsSpinBox->blockSignals(true);
    planEpsilonStepsSpinBox->setValue(plannerConfig.epsilonSteps);
    planEpsilonStepsSpinBox->blockSignals(wasBlockedPlanEpsSteps);

    const bool wasBlockedPlanInitEps = planInitialEpsilonSpinBox->blockSignals(true);
    planInitialEpsilonSpinBox->setValue(plannerConfig.initialEpsilon);
    planInitialEpsilonSpinBox->blockSignals(wasBlockedPlanInitEps);

    const bool wasBlockedPlanPathStats = planUsePathStatisticsCheckBox->blockSignals(true);
    planUsePathStatisticsCheckBox->setChecked(plannerConfig.usePathStatistics);
    planUsePathStatisticsCheckBox->blockSignals(wasBlockedPlanPathStats);

    const bool wasBlockedPlanSearchUntil = planSearchUntilFirstSolutionCheckBox->blockSignals(true);
    planSearchUntilFirstSolutionCheckBox->setChecked(plannerConfig.searchUntilFirstSolution);
    planSearchUntilFirstSolutionCheckBox->blockSignals(wasBlockedPlanSearchUntil);

    // Cost multipliers
    const bool wasBlockedMultForward = mobMultForwardSpinBox->blockSignals(true);
    mobMultForwardSpinBox->setValue(mobilityConfig.multiplierForward);
    mobMultForwardSpinBox->blockSignals(wasBlockedMultForward);

    const bool wasBlockedMultBackward = mobMultBackwardSpinBox->blockSignals(true);
    mobMultBackwardSpinBox->setValue(mobilityConfig.multiplierBackward);
    mobMultBackwardSpinBox->blockSignals(wasBlockedMultBackward);

    const bool wasBlockedMultLateral = mobMultLateralSpinBox->blockSignals(true);
    mobMultLateralSpinBox->setValue(mobilityConfig.multiplierLateral);
    mobMultLateralSpinBox->blockSignals(wasBlockedMultLateral);

    const bool wasBlockedMultForwardTurn = mobMultForwardTurnSpinBox->blockSignals(true);
    mobMultForwardTurnSpinBox->setValue(mobilityConfig.multiplierForwardTurn);
    mobMultForwardTurnSpinBox->blockSignals(wasBlockedMultForwardTurn);

    const bool wasBlockedMultBackwardTurn = mobMultBackwardTurnSpinBox->blockSignals(true);
    mobMultBackwardTurnSpinBox->setValue(mobilityConfig.multiplierBackwardTurn);
    mobMultBackwardTurnSpinBox->blockSignals(wasBlockedMultBackwardTurn);

    const bool wasBlockedMultPoint = mobMultPointTurnSpinBox->blockSignals(true);
    mobMultPointTurnSpinBox->setValue(mobilityConfig.multiplierPointTurn);
    mobMultPointTurnSpinBox->blockSignals(wasBlockedMultPoint);

    const bool wasBlockedMultLateralCurve = mobMultLateralCurveSpinBox->blockSignals(true);
    mobMultLateralCurveSpinBox->setValue(mobilityConfig.multiplierLateralCurve);
    mobMultLateralCurveSpinBox->blockSignals(wasBlockedMultLateralCurve);
}

void PlannerGui::replanButtonReleased()
{
    plannerHasRun = false;
    startPlanThread();       
}

void PlannerGui::updateParamsButtonReleased()
{
    LOG_INFO_S << "Updating underlying structures with new parameters...";

    std::cout << "\n========================================\n"
              << "Planner GUI Configuration Update:\n"
              << "----------------------------------------\n"
              << "Spline Configuration:\n"
              << "  Grid Size: " << splineConfig.gridSize << " m\n"
              << "  Num Angles: " << splineConfig.numAngles << "\n"
              << "  Num End Angles: " << splineConfig.numEndAngles << "\n"
              << "  Destination Circle Radius: " << splineConfig.destinationCircleRadius << "\n"
              << "  Cell Skip Factor: " << splineConfig.cellSkipFactor << "\n"
              << "  Generate Point Turns: " << (splineConfig.generatePointTurnMotions ? "Yes" : "No") << "\n"
              << "  Generate Lateral: " << (splineConfig.generateLateralMotions ? "Yes" : "No") << "\n"
              << "  Generate Backward: " << (splineConfig.generateBackwardMotions ? "Yes" : "No") << "\n"
              << "  Generate Forward: " << (splineConfig.generateForwardMotions ? "Yes" : "No") << "\n"
              << "  Spline Order: " << splineConfig.splineOrder << "\n"
              << "----------------------------------------\n"
              << "Mobility Configuration:\n"
              << "  Translation Speed: " << mobilityConfig.translationSpeed << " m/s\n"
              << "  Rotation Speed: " << mobilityConfig.rotationSpeed << " rad/s\n"
              << "  Min Turning Radius: " << mobilityConfig.minTurningRadius << " m\n"
              << "  Curvature Penalty Weight: " << mobilityConfig.curvaturePenaltyWeight << "\n"
              << "  Angular Cost Weight: " << mobilityConfig.angularCostWeight << "\n"
              << "  Search Radius: " << mobilityConfig.searchRadius << " m\n"
              << "  Search Progress Steps: " << mobilityConfig.searchProgressSteps << " m\n"
              << "  Max Motion Curve Length: " << mobilityConfig.maxMotionCurveLength << "\n"
              << "  Spline Sampling Resolution: " << mobilityConfig.spline_sampling_resolution << " m\n"
              << "  Remove Goal Offset: " << (mobilityConfig.remove_goal_offset ? "Yes" : "No") << "\n"
              << "  Cost Multipliers:\n"
              << "    Forward: " << mobilityConfig.multiplierForward << "\n"
              << "    Forward Turn: " << mobilityConfig.multiplierForwardTurn << "\n"
              << "    Backward: " << mobilityConfig.multiplierBackward << "\n"
              << "    Backward Turn: " << mobilityConfig.multiplierBackwardTurn << "\n"
              << "    Lateral: " << mobilityConfig.multiplierLateral << "\n"
              << "    Lateral Curve: " << mobilityConfig.multiplierLateralCurve << "\n"
              << "    Point Turn: " << mobilityConfig.multiplierPointTurn << "\n"
              << "----------------------------------------\n"
              << "Traversability Configuration:\n"
              << "  Grid Resolution: " << travConfig.gridResolution << " m\n"
              << "  Max Slope: " << (travConfig.maxSlope * 180.0 / M_PI) << " deg\n"
              << "  Max Step Height: " << travConfig.maxStepHeight << " m\n"
              << "  Robot Size: " << travConfig.robotSizeX << " x " << travConfig.robotSizeY << " x " << travConfig.robotHeight << " m\n"
              << "  Distance to Ground: " << travConfig.distToGround << " m\n"
              << "  Obstacle Inflation Multiplier: " << travConfig.obstacleInflationMultiplier << "\n"

              << "  Min Traversable Percentage: " << travConfig.minTraversablePercentage << "\n"
              << "  Allow Forward Downhill: " << (travConfig.allowForwardDownhill ? "Yes" : "No") << "\n"
              << "  Enable Incline Limiting: " << (travConfig.enableInclineLimitting ? "Yes" : "No") << "\n"
              << "----------------------------------------\n"
              << "Planner/Search Configuration:\n"
              << "  Initial Epsilon: " << plannerConfig.initialEpsilon << "\n"
              << "  Epsilon Steps: " << plannerConfig.epsilonSteps << "\n"
              << "  Num Threads: " << plannerConfig.numThreads << "\n"
              << "  Use Path Statistics: " << (plannerConfig.usePathStatistics ? "Yes" : "No") << "\n"
              << "  Search Until First Solution: " << (plannerConfig.searchUntilFirstSolution ? "Yes" : "No") << "\n"
              << "========================================\n" << std::endl;

    std::shared_ptr<const traversability_generator3d::TravMap3d> oldMap;
    if (usingPlannerDump && planner)
    {
        oldMap = planner->getTraversabilityMap();
    }

    if (!usingPlannerDump)
    {
        if (travGen)
        {
            travGen.reset(new traversability_generator3d::TraversabilityGenerator3d(travConfig));
            std::shared_ptr<maps::grid::MLSMapSloped> mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mlsMap);
            travGen->setMLSGrid(mlsPtr);
        }
    }

    plannerHasRun = false;
    planner.reset(new ugv_nav4d::Planner(splineConfig, travConfig, mobilityConfig, plannerConfig));

    if (usingPlannerDump && oldMap)
    {
        planner->updateMap(*oldMap);
    }

    // Re-generate visualizer spline primitive paths using updated configs
    sbpl_spline_primitives::SbplSplineMotionPrimitives primitives(splineConfig);
    splineViz.setMaxCurvature(ugv_nav4d::PreComputedMotions::calculateCurvatureFromRadius(mobilityConfig.minTurningRadius));
    splineViz.updateData(primitives);
    LOG_INFO_S << "Underlying structures and motion primitives updated successfully!";
}

// Spline slots
void PlannerGui::splineGridSizeEditingFinished() { splineConfig.gridSize = splineGridSizeSpinBox->value(); }
void PlannerGui::splineNumAnglesValueChanged(int value) { splineConfig.numAngles = value; }
void PlannerGui::splineNumEndAnglesValueChanged(int value) { splineConfig.numEndAngles = value; }
void PlannerGui::splineDestCircleRadiusEditingFinished() { splineConfig.destinationCircleRadius = splineDestCircleRadiusSpinBox->value(); }
void PlannerGui::splineCellSkipFactorEditingFinished() { splineConfig.cellSkipFactor = splineCellSkipFactorSpinBox->value(); }
void PlannerGui::splineGenPointTurnMotionsStateChanged(int state) { splineConfig.generatePointTurnMotions = (state == Qt::Checked); }
void PlannerGui::splineGenLateralMotionsStateChanged(int state) { splineConfig.generateLateralMotions = (state == Qt::Checked); }
void PlannerGui::splineGenBackwardMotionsStateChanged(int state) { splineConfig.generateBackwardMotions = (state == Qt::Checked); }
void PlannerGui::splineGenForwardMotionsStateChanged(int state) { splineConfig.generateForwardMotions = (state == Qt::Checked); }
void PlannerGui::splineOrderValueChanged(int value) { splineConfig.splineOrder = value; }

// Mobility slots
void PlannerGui::mobSearchRadiusEditingFinished() { mobilityConfig.searchRadius = mobSearchRadiusSpinBox->value(); }
void PlannerGui::mobSearchProgressStepsEditingFinished() { mobilityConfig.searchProgressSteps = mobSearchProgressStepsSpinBox->value(); }
void PlannerGui::mobMultForwardValueChanged(int value) { mobilityConfig.multiplierForward = value; }
void PlannerGui::mobMultBackwardValueChanged(int value) { mobilityConfig.multiplierBackward = value; }
void PlannerGui::mobMultLateralValueChanged(int value) { mobilityConfig.multiplierLateral = value; }
void PlannerGui::mobMultForwardTurnValueChanged(int value) { mobilityConfig.multiplierForwardTurn = value; }
void PlannerGui::mobMultBackwardTurnValueChanged(int value) { mobilityConfig.multiplierBackwardTurn = value; }
void PlannerGui::mobMultPointTurnValueChanged(int value) { mobilityConfig.multiplierPointTurn = value; }
void PlannerGui::mobMultLateralCurveValueChanged(int value) { mobilityConfig.multiplierLateralCurve = value; }
void PlannerGui::mobMaxMotionCurveLengthEditingFinished() { mobilityConfig.maxMotionCurveLength = mobMaxMotionCurveLengthSpinBox->value(); }
void PlannerGui::mobSplineSamplingResEditingFinished() { mobilityConfig.spline_sampling_resolution = mobSplineSamplingResSpinBox->value(); }
void PlannerGui::mobRemoveGoalOffsetStateChanged(int state) { mobilityConfig.remove_goal_offset = (state == Qt::Checked); }
void PlannerGui::mobCurvaturePenaltyWeightEditingFinished() { mobilityConfig.curvaturePenaltyWeight = mobCurvaturePenaltyWeightSpinBox->value(); }
void PlannerGui::mobAngularCostWeightEditingFinished() { mobilityConfig.angularCostWeight = mobAngularCostWeightSpinBox->value(); }

// Traversability slots
void PlannerGui::travGridResolutionEditingFinished() { travConfig.gridResolution = travGridResolutionSpinBox->value(); }
void PlannerGui::travMaxStepHeightEditingFinished() { travConfig.maxStepHeight = travMaxStepHeightSpinBox->value(); }
void PlannerGui::travMinTraversablePercentageEditingFinished() { travConfig.minTraversablePercentage = travMinTraversablePercentageSpinBox->value(); }
void PlannerGui::travAllowForwardDownhillStateChanged(int state) { travConfig.allowForwardDownhill = (state == Qt::Checked); }
void PlannerGui::travEnableInclineLimittingStateChanged(int state) { travConfig.enableInclineLimitting = (state == Qt::Checked); }
void PlannerGui::travObstacleInflationMultiplierEditingFinished() { travConfig.obstacleInflationMultiplier = travObstacleInflationMultiplierSpinBox->value(); }


// Planner slots
void PlannerGui::planEpsilonStepsEditingFinished() { plannerConfig.epsilonSteps = planEpsilonStepsSpinBox->value(); }
void PlannerGui::planInitialEpsilonEditingFinished() { plannerConfig.initialEpsilon = planInitialEpsilonSpinBox->value(); }
void PlannerGui::planUsePathStatisticsStateChanged(int state) { plannerConfig.usePathStatistics = (state == Qt::Checked); }
void PlannerGui::planSearchUntilFirstSolutionStateChanged(int state) { plannerConfig.searchUntilFirstSolution = (state == Qt::Checked); }

void PlannerGui::startPlanThread()
{

    bar->setMaximum(0);

    // Check if planning is already in progress
    if (inplanningphase.load()) {
        std::cout << "Planner is in planning phase... Please wait for it to finish." << std::endl;
        return;
    }

    // Mark the start of the planning phase
    inplanningphase.store(true);    
    
    std::thread t([this](){
#ifdef ENABLE_DEBUG_DRAWINGS
        V3DD::CONFIGURE_DEBUG_DRAWINGS_USE_EXISTING_WIDGET(this->widget);
#endif
        if (!usingPlannerDump){
            std::vector<Eigen::Vector3d> startPositions;
            startPositions.emplace_back(Eigen::Vector3d(this->start.position.x(),
                                                        this->start.position.y(),
                                                        this->start.position.z()-travConfig.distToGround));

            travGen->expandAll(startPositions);
            planner->updateMap(travGen->getTraversabilityMap());
        }
        this->plan(this->start, this->goal);

        // Mark the end of the planning phase after work is done
        inplanningphase.store(false);

    });
    t.detach(); //needed to avoid destruction of thread at end of method
}


void PlannerGui::plannerIsDone()
{   
    trajViz.updateData(path);
    trajViz.setLineWidth(8);

    trajViz2.updateData(beautifiedPath);
    trajViz2.setLineWidth(8);    
    
    trav3dViz.updateData(*(planner->getTraversabilityMap()));
    
    bar->setMaximum(1);
    plannerHasRun = true;
}

void PlannerGui::dumpPressed()
{
    if (!plannerHasRun && !usingPlannerDump)
    {
        LOG_WARN_S << "Cannot create PlannerDump: Planner has not run yet or map is empty.";
        return;
    }
    LOG_INFO_S << "Dumping";
    
    base::samples::RigidBodyState startState;
    startState.position = start.position;
    startState.orientation = start.orientation;
    base::samples::RigidBodyState endState;
    endState.position << goal.position;
    endState.orientation = goal.orientation;
    
    PlannerDump dump(*planner, "created_by_test_gui", base::Time::fromSeconds(time->value()),
                     startState, endState);
}



/* 
Start:  5.99972 0.399847 -1.31341d
goal: -0.455198   7.99133   2.08586
*/

void PlannerGui::plan(const base::Pose& start, const base::Pose& goal)
{   
    base::samples::RigidBodyState startState;
    startState.position = start.position;
    startState.orientation = start.orientation;
    base::samples::RigidBodyState endState;
    endState.position << goal.position;
    endState.orientation = goal.orientation;

    LOG_INFO_S << "Planning: " << start << " -> " << goal;
    
    const Planner::PLANNING_RESULT result = planner->plan(base::Time::fromSeconds(time->value()),
                                            startState, endState, path, beautifiedPath);
    switch(result)
    {
        case Planner::GOAL_INVALID:
            LOG_INFO_S << "GOAL_INVALID";
            break;
        case Planner::START_INVALID:
            LOG_INFO_S << "START_INVALID";
            break; 
        case Planner::NO_SOLUTION:
            LOG_INFO_S << "NO_SOLUTION";
            break;
       case Planner::NO_MAP:
            LOG_INFO_S << "NO_MAP";
            break;
        case Planner::INTERNAL_ERROR:
            LOG_INFO_S << "INTERNAL_ERROR";
            break;
        case Planner::FOUND_SOLUTION:
            LOG_INFO_S << "FOUND_SOLUTION";
            break;
        default:
            LOG_INFO_S << "ERROR unknown result state";
            break;
    }
    
    emit plannerDone();
}




