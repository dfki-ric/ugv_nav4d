#include "PlannerGui.h"
#include <QFileDialog>
#include <QSpinBox>
#include <QPushButton>
#include <QProgressBar>
#include <QLabel>
#include <QSlider>
#include <QComboBox>
#include <QHBoxLayout>
#include <thread>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <ugv_nav4d/PreComputedMotions.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>
#include <vizkit3d_debug_drawings/DebugDrawingColors.hpp>
#include <boost/filesystem.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <ugv_nav4d/PlannerDump.hpp>
#include <pcl/common/transforms.h>
#include <base-logging/Logging.hpp>

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
    
    sbpl_spline_primitives::SbplSplineMotionPrimitives primitives(splineConfig);
    splineViz.setMaxCurvature(ugv_nav4d::PreComputedMotions::calculateCurvatureFromRadius(mobilityConfig.minTurningRadius));
    splineViz.updateData(primitives);

    start = dump.getStart().getPose();
    goal = dump.getGoal().getPose();

    startViz.updateData(dump.getStart());
    goalViz.updateData(dump.getGoal());

    planner->updateMap(dump.getMlsMap());

    inplanningphase = false;
}


PlannerGui::PlannerGui(int argc, char** argv): QObject()
{
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
    obstacleMapViz.setPluginName("ObstacleMap");
    
    startViz.setPluginName("Start Pose");
    goalViz.setPluginName("Goal Pose");

    widget->setCameraManipulator(vizkit3d::ORBIT_MANIPULATOR);
    widget->addPlugin(&splineViz);
    widget->addPlugin(&trajViz);
    widget->addPlugin(&trajViz2);
    widget->addPlugin(&mlsViz);
    widget->addPlugin(&trav3dViz);
    widget->addPlugin(&obstacleMapViz);
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
    
    maxSlopeSpinBox = new QDoubleSpinBox();
    maxSlopeSpinBox->setMinimum(1);
    maxSlopeSpinBox->setMaximum(60);
    maxSlopeSpinBox->setValue(33.23);

    connect(maxSlopeSpinBox, SIGNAL(editingFinished()), this, SLOT(maxSlopeEditingFinished()));
    QHBoxLayout* slopeLayout = new QHBoxLayout();
    QLabel* lab = new QLabel();
    lab->setText("max slope (deg):");

    slopeLayout->addWidget(lab);
    slopeLayout->addWidget(maxSlopeSpinBox);
    //layout->addLayout(slopeLayout);
    
    QHBoxLayout* timeLayout = new QHBoxLayout();
    time = new QDoubleSpinBox();
    time->setMinimum(1);
    time->setMaximum(9999999);
    time->setValue(14);
    QLabel* lab2 = new QLabel();
    lab2->setText("Max processor time (seconds)");
    timeLayout->addWidget(lab2);
    timeLayout->addWidget(time);
    timeLayout->addWidget(time);
    layout->addLayout(timeLayout);
    connect(time, SIGNAL(editingFinished()), this, SLOT(timeEditingFinished()));
    
    bar = new QProgressBar();
    bar->setMinimum(0);
    bar->setMaximum(1);

    inclineLimittingMinSlopeSpinBox = new QDoubleSpinBox();
    inclineLimittingMinSlopeSpinBox->setMinimum(0.0);
    inclineLimittingMinSlopeSpinBox->setMaximum(180.0);
    inclineLimittingMinSlopeSpinBox->setValue(20);
    connect(inclineLimittingMinSlopeSpinBox, SIGNAL(editingFinished()), this, SLOT(inclineLimittingMinSlopeSpinBoxEditingFinished()));
    
    inclineLimittingLimitSpinBox = new QDoubleSpinBox();
    inclineLimittingLimitSpinBox->setMinimum(0.00001);
    inclineLimittingLimitSpinBox->setMaximum(90);    
    inclineLimittingLimitSpinBox->setValue(25.21);
    connect(inclineLimittingLimitSpinBox, SIGNAL(editingFinished()), this, SLOT(inclineLimittingLimitSpinBoxEditingFinished()));
    
    QLabel* inclineLimittingMinSlopeLabel = new QLabel();
    inclineLimittingMinSlopeLabel->setText("Incline limit min slope (deg)");
    QLabel* inclineLimittingLimitSpinBoxLabel = new QLabel();
    inclineLimittingLimitSpinBoxLabel->setText("Incline limit at max slope (deg)");
    
    QHBoxLayout* slopeLimitLayout = new QHBoxLayout();
    slopeLimitLayout->addWidget(inclineLimittingMinSlopeLabel);
    slopeLimitLayout->addWidget(inclineLimittingMinSlopeSpinBox);
    QHBoxLayout* slopeLimitLayout2 = new QHBoxLayout();
    slopeLimitLayout2->addWidget(inclineLimittingLimitSpinBoxLabel);
    slopeLimitLayout2->addWidget(inclineLimittingLimitSpinBox);
    
    //layout->addLayout(slopeLimitLayout);
    //layout->addLayout(slopeLimitLayout2);
    
    
    slopeMetricScaleSpinBox = new QDoubleSpinBox();
    slopeMetricScaleSpinBox->setMinimum(0.0);
    slopeMetricScaleSpinBox->setMaximum(999999.0);
    slopeMetricScaleSpinBox->setValue(0.0);
    connect(slopeMetricScaleSpinBox, SIGNAL(editingFinished()), this, SLOT(slopeMetricScaleSpinBoxEditingFinished()));
    
    QLabel* slopeMetricLabel = new QLabel();
    slopeMetricLabel->setText("slope metric scale");
    
    QHBoxLayout* slopeMetricLayout = new QHBoxLayout();
    slopeMetricLayout->addWidget(slopeMetricLabel);
    slopeMetricLayout->addWidget(slopeMetricScaleSpinBox);
    //layout->addLayout(slopeMetricLayout);
    
    slopeMetricComboBox = new QComboBox();
    slopeMetricComboBox->addItem("NONE");
    slopeMetricComboBox->addItem("AVG_SLOPE");
    slopeMetricComboBox->addItem("MAX_SLOPE");
    slopeMetricComboBox->addItem("TRIANGLE_SLOPE");
    connect(slopeMetricComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(slopeMetricComboBoxIndexChanged(int)));
    QLabel* slopeMetricComboLabel = new QLabel();
    slopeMetricComboLabel->setText("Slope Metric Type");
    QHBoxLayout* slopeMetricTypeLayout = new QHBoxLayout();
    slopeMetricTypeLayout->addWidget(slopeMetricComboLabel);
    slopeMetricTypeLayout->addWidget(slopeMetricComboBox);
    //layout->addLayout(slopeMetricTypeLayout);
    
    startOrientatationSlider = new QSlider(Qt::Horizontal);
    startOrientatationSlider->setMinimum(0);
    startOrientatationSlider->setMaximum(359);
    startOrientatationSlider->setValue(0);
    goalOrientationSlider = new QSlider(Qt::Horizontal);
    goalOrientationSlider->setMinimum(0);
    goalOrientationSlider->setMaximum(359);
    goalOrientationSlider->setValue(0);
    
    connect(startOrientatationSlider, SIGNAL(sliderMoved(int)), this, SLOT(startOrientationChanged(int)));
    connect(goalOrientationSlider, SIGNAL(sliderMoved(int)), this, SLOT(goalOrientationChanged(int)));
    
    QLabel* startOrientationLaebel = new QLabel();
    startOrientationLaebel->setText("Start orientation (deg)");
    QLabel* goalOrientationLaebel = new QLabel();
    goalOrientationLaebel->setText("Goal orientation (deg)");
     
    QHBoxLayout* startOrientationLayout = new QHBoxLayout();
    startOrientationLayout->addWidget(startOrientationLaebel);
    startOrientationLayout->addWidget(startOrientatationSlider);
    
    QHBoxLayout* goalOrientationLayout = new QHBoxLayout();
    goalOrientationLayout->addWidget(goalOrientationLaebel);
    goalOrientationLayout->addWidget(goalOrientationSlider);
    
    layout->addLayout(startOrientationLayout);
    layout->addLayout(goalOrientationLayout);
    
    obstacleDistanceSpinBox = new QDoubleSpinBox();
    obstacleDistanceSpinBox->setMaximum(99999);
    obstacleDistanceSpinBox->setMinimum(0);
    obstacleDistanceSpinBox->setValue(0.0);
    
    obstacleFactorSpinBox = new QDoubleSpinBox();
    obstacleFactorSpinBox->setMinimum(0);
    obstacleFactorSpinBox->setMaximum(9999999);
    obstacleFactorSpinBox->setValue(1.0);

    QLabel* obstacleDistanceLable = new QLabel();
    obstacleDistanceLable->setText("Obstacle Distance");
    QHBoxLayout* obstacleDistLayout = new QHBoxLayout();
    obstacleDistLayout->addWidget(obstacleDistanceLable);
    obstacleDistLayout->addWidget(obstacleDistanceSpinBox);
    
    QLabel* obstacleFactorLabel = new QLabel();
    obstacleFactorLabel->setText("Obstacle cost factor");
    QHBoxLayout* obstacleFactorLayout = new QHBoxLayout();
    obstacleFactorLayout->addWidget(obstacleFactorLabel);
    obstacleFactorLayout->addWidget(obstacleFactorSpinBox);
    
    //layout->addLayout(obstacleDistLayout);
    //layout->addLayout(obstacleFactorLayout);
    
    connect(obstacleDistanceSpinBox, SIGNAL(editingFinished()), this, SLOT(obstacleDistanceSpinBoxEditingFinished()));
    connect(obstacleFactorSpinBox, SIGNAL(editingFinished()), this, SLOT(obstacleFactorSpinBoxEditingFinished()));
    
    numThreadsSpinBox = new QSpinBox();
    numThreadsSpinBox->setValue(4);
    QLabel* parallelismLabel = new QLabel();
    parallelismLabel->setText("Number of Threads");
    
    QHBoxLayout* parallelismLayout = new QHBoxLayout();
    parallelismLayout->addWidget(parallelismLabel);
    parallelismLayout->addWidget(numThreadsSpinBox);
    
    layout->addLayout(parallelismLayout);
    connect(numThreadsSpinBox, SIGNAL(valueChanged(int)), this, SLOT(numThreadsValueChanged(int)));
    
    
    QPushButton* replanButton = new QPushButton("Plan");
    timeLayout->addWidget(replanButton);
    
    QPushButton* dumpButton = new QPushButton("Create PlannerDump");
    timeLayout->addWidget(dumpButton);

    
    connect(replanButton, SIGNAL(released()), this, SLOT(replanButtonReleased()));
    connect(dumpButton, SIGNAL(released()), this, SLOT(dumpPressed()));
    
    
    layout->addWidget(bar);
    
    window.setLayout(layout);

    //to be able to send Trajectory via slot
    qRegisterMetaType<std::vector<ugv_nav4d::Motion>>("std::vector<ugv_nav4d::Motion>");
    qRegisterMetaType<std::vector<base::Trajectory>>("std::vector<base::Trajectory>");
    qRegisterMetaType<maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase*>>("maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase*>");

    connect(&mlsViz, SIGNAL(picked(float,float,float, int, int)), this, SLOT(picked(float,float,float, int, int)));
    connect(&trav3dViz, SIGNAL(picked(float,float,float, int, int)), this, SLOT(picked(float,float,float, int, int)));
    connect(&obstacleMapViz, SIGNAL(picked(float,float,float, int, int)), this, SLOT(picked(float,float,float, int, int)));
    connect(this, SIGNAL(plannerDone()), this, SLOT(plannerIsDone()));

}


void PlannerGui::setupPlanner(int argc, char** argv)
{
    double res = 0.3;
    if(argc > 2){
        std::setlocale(LC_ALL, "C");
        res = atof(argv[2]);
    }

    splineConfig.gridSize = res;
    splineConfig.numAngles = 16;
    splineConfig.numEndAngles = 8;
    splineConfig.destinationCircleRadius = 6;
    splineConfig.cellSkipFactor = 0.1;
    splineConfig.generatePointTurnMotions = true;
    splineConfig.generateLateralMotions = true;
    splineConfig.generateBackwardMotions = true;
    splineConfig.generateForwardMotions = true;
    splineConfig.splineOrder = 4;

    mobilityConfig.translationSpeed = 0.5;
    mobilityConfig.rotationSpeed = 0.5;
    mobilityConfig.minTurningRadius = 1; // increase this to reduce the number of available motion primitives
    mobilityConfig.searchRadius = 1.0;
    mobilityConfig.searchProgressSteps = 0.1;
    mobilityConfig.multiplierForward = 1;
    mobilityConfig.multiplierForwardTurn = 2;
    mobilityConfig.multiplierBackward = 2;
    mobilityConfig.multiplierBackwardTurn = 3;
    mobilityConfig.multiplierLateral = 4;
    mobilityConfig.multiplierLateralCurve = 4;
    mobilityConfig.multiplierPointTurn = 3;
    mobilityConfig.maxMotionCurveLength = 100;
    mobilityConfig.spline_sampling_resolution = 0.05;
    mobilityConfig.remove_goal_offset = false;

    travConfig.gridResolution = res;
    travConfig.maxSlope = 0.45; //40.0/180.0 * M_PI;
    travConfig.maxStepHeight = 0.20; //space below robot
    travConfig.robotSizeX = 0.5;
    travConfig.robotSizeY =  0.5;
    travConfig.robotHeight = 0.5; //incl space below body
    travConfig.slopeMetricScale = 1.0;
    travConfig.slopeMetric = traversability_generator3d::SlopeMetric::NONE;
    travConfig.inclineLimittingMinSlope = 0.22; // 10.0 * M_PI/180.0;
    travConfig.inclineLimittingLimit = 0.43;// 5.0 * M_PI/180.0;
    travConfig.costFunctionDist = 0.0;
    travConfig.distToGround = 0.0;
    travConfig.minTraversablePercentage = 0.5;
    travConfig.allowForwardDownhill = true;
    travConfig.enableInclineLimitting = false;

    plannerConfig.epsilonSteps = 2.0;
    plannerConfig.initialEpsilon = 64.0;
    plannerConfig.numThreads = 4;

    planner.reset(new ugv_nav4d::Planner(splineConfig, travConfig, mobilityConfig, plannerConfig));

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
    
    if(argc > 3)
    {
        start.position << 9.08776,  5.8535, 0.06052;
        QVector3D pos(start.position.x(), start.position.y(), start.position.z());
        startViz.setTranslation(pos);

        
        LOG_INFO_S << "Start: " << start.position.transpose();
        pickStart = false;
    }
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
            planner->updateMap(mlsMap);
        }
        return;
    }

    
    
    try
    {
        LOG_INFO_S << "Loading MLS";
        boost::archive::binary_iarchive mlsIn(fileIn);
        mlsIn >> mlsMap;
        mlsViz.updateMLSSloped(mlsMap);
        planner->updateMap(mlsMap);
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

void PlannerGui::replanButtonReleased()
{
    planner->setTravConfig(travConfig);
    planner->setPlannerConfig(plannerConfig);
    startPlanThread();       
}

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
    
    trav3dViz.updateData((planner->getTraversabilityMap().copyCast<maps::grid::TraversabilityNodeBase *>()));
    obstacleMapViz.updateData((planner->getObstacleMap().copyCast<maps::grid::TraversabilityNodeBase *>()));
    
    bar->setMaximum(1);
}

void PlannerGui::dumpPressed()
{
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




