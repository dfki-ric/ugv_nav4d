#include "PlannerGui.h"
#include <QFileDialog>
#include <QPushButton>
#include <thread>
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
    
    startPlanThread();
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
    V3DD::CONFIGURE_DEBUG_DRAWINGS_USE_EXISTING_WIDGET(widget);
    
    trav3dViz.setPluginName("TravMap");
    obstacleMapViz.setPluginName("ObstacleMap");
    
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
    gridViz.setPluginEnabled(true);
    
    mlsViz.setCycleHeightColor(true);
    mlsViz.setShowPatchExtents(false); 
    mlsViz.setShowNormals(false);
    
    trajViz.setLineWidth(5);
    trajViz.setColor(QColor("Cyan"));
    trajViz2.setLineWidth(5);
    trajViz2.setColor(QColor("magenta"));
        
    QVBoxLayout* layout = new QVBoxLayout();
    
       
    layout->addWidget(widget);
    
    maxSlopeSpinBox = new QDoubleSpinBox();
    maxSlopeSpinBox->setMinimum(1);
    maxSlopeSpinBox->setMaximum(60);
    connect(maxSlopeSpinBox, SIGNAL(editingFinished()), this, SLOT(maxSlopeEditingFinished()));
    QHBoxLayout* slopeLayout = new QHBoxLayout();
    QLabel* lab = new QLabel();
    lab->setText("max slope (deg):");
    slopeLayout->addWidget(lab);
    slopeLayout->addWidget(maxSlopeSpinBox);
    layout->addLayout(slopeLayout);
    
    QHBoxLayout* timeLayout = new QHBoxLayout();
    time = new QDoubleSpinBox();
    time->setMinimum(1);
    time->setMaximum(9999999);
    time->setValue(14);
    QLabel* lab2 = new QLabel();
    lab2->setText("Max processor time:");
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
    connect(inclineLimittingMinSlopeSpinBox, SIGNAL(editingFinished()), this, SLOT(inclineLimittingMinSlopeSpinBoxEditingFinished()));
    
    inclineLimittingLimitSpinBox = new QDoubleSpinBox();
    inclineLimittingLimitSpinBox->setMinimum(0.00001);
    inclineLimittingLimitSpinBox->setMaximum(90);
    connect(inclineLimittingLimitSpinBox, SIGNAL(editingFinished()), this, SLOT(inclineLimittingLimitSpinBoxEditingFinished()));
    
    QLabel* inclineLimittingMinSlopeLabel = new QLabel();
    inclineLimittingMinSlopeLabel->setText("incline limit min slope (deg)");
    QLabel* inclineLimittingLimitSpinBoxLabel = new QLabel();
    inclineLimittingLimitSpinBoxLabel->setText("incline limit at max slope (deg)");
    
    QHBoxLayout* slopeLimitLayout = new QHBoxLayout();
    slopeLimitLayout->addWidget(inclineLimittingMinSlopeLabel);
    slopeLimitLayout->addWidget(inclineLimittingMinSlopeSpinBox);
    QHBoxLayout* slopeLimitLayout2 = new QHBoxLayout();
    slopeLimitLayout2->addWidget(inclineLimittingLimitSpinBoxLabel);
    slopeLimitLayout2->addWidget(inclineLimittingLimitSpinBox);
    
    layout->addLayout(slopeLimitLayout);
    layout->addLayout(slopeLimitLayout2);
    
    
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
    layout->addLayout(slopeMetricLayout);
    
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
    layout->addLayout(slopeMetricTypeLayout);
    
    
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
    startOrientationLaebel->setText("start orientation (deg)");
    QLabel* goalOrientationLaebel = new QLabel();
    goalOrientationLaebel->setText("goal orientation (deg)");
     
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
    obstacleDistanceSpinBox->setValue(0.4);
    
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
    
    layout->addLayout(obstacleDistLayout);
    layout->addLayout(obstacleFactorLayout);
    
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
    
    maxSlopeSpinBox->setValue(33.23);
    inclineLimittingMinSlopeSpinBox->setValue(20);
    inclineLimittingLimitSpinBox->setValue(25.21);
}


void PlannerGui::setupPlanner(int argc, char** argv)
{
    double res = 0.1;
     if(argc > 2)
         res = atof(argv[2]);
    
    splineConfig.gridSize = res;
    splineConfig.numAngles = 24;
    splineConfig.numEndAngles = 12;
    splineConfig.destinationCircleRadius = 5;
    splineConfig.cellSkipFactor = 1.0;
    splineConfig.generatePointTurnMotions = true;
    splineConfig.generateLateralMotions = false;
    splineConfig.generateBackwardMotions = true;
    splineConfig.splineOrder = 4;
    
    mobilityConfig.translationSpeed = 0.2;
    mobilityConfig.rotationSpeed = 0.6;
    mobilityConfig.minTurningRadius = 0.2; // increase this to reduce the number of available motion primitives
    mobilityConfig.searchRadius = 0.0;
    
    mobilityConfig.multiplierForward = 1;
    mobilityConfig.multiplierBackward = 1;
    mobilityConfig.multiplierLateral = 3;
    mobilityConfig.multiplierBackwardTurn = 1;
    mobilityConfig.multiplierForwardTurn = 1;
    mobilityConfig.multiplierPointTurn = 3;
     
    travConfig.gridResolution = res;
    travConfig.maxSlope = 0.57; //40.0/180.0 * M_PI;
    travConfig.maxStepHeight = 0.3; //space below robot
    travConfig.robotSizeX = 0.3;
    travConfig.robotSizeY =  0.3;
    travConfig.robotHeight = 0.3; //incl space below body
    travConfig.slopeMetricScale = 0.0;
    travConfig.slopeMetric = traversability_generator3d::SlopeMetric::NONE;
    travConfig.inclineLimittingMinSlope = 0.22; // 10.0 * M_PI/180.0;
    travConfig.inclineLimittingLimit = 0.43;// 5.0 * M_PI/180.0;
    travConfig.costFunctionDist = 0.4;
    travConfig.distToGround = 0.2;
    travConfig.minTraversablePercentage = 0.5;
    travConfig.allowForwardDownhill = true;
    plannerConfig.epsilonSteps = 2.0;
    plannerConfig.initialEpsilon = 20.0;
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

            //transform point cloud to zero (instead we could also use MlsMap::translate later but that seems to be broken?)
            Eigen::Affine3f pclTf = Eigen::Affine3f::Identity();
            pclTf.translation() << -mi.x, -mi.y, -mi.z;
            pcl::transformPointCloud (*cloud, *cloud, pclTf);
            
            pcl::getMinMax3D (*cloud, mi, ma); 
            LOG_INFO_S << "MIN: " << mi << ", MAX: " << ma;
        
            const double mls_res = travConfig.gridResolution;
            const double size_x = ma.x;
            const double size_y = ma.y;
            
            const maps::grid::Vector2ui numCells(size_x / mls_res + 1, size_y / mls_res + 1);
            LOG_INFO_S << "NUM CELLS: " << numCells;
            
            maps::grid::MLSConfig cfg;
            cfg.gapSize = 0.1;
            mlsMap = maps::grid::MLSMapSloped(numCells, maps::grid::Vector2d(mls_res, mls_res), cfg);
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
            
            V3DD::CLEAR_DRAWING("ugv_nav4d_start_aabb");
            V3DD::DRAW_WIREFRAME_BOX("ugv_nav4d_start_aabb", start.position +  base::Vector3d(0, 0, travConfig.distToGround / 2.0), start.orientation,
                               base::Vector3d(travConfig.robotSizeX, travConfig.robotSizeY, travConfig.robotHeight - travConfig.distToGround), V3DD::Color::cyan);
            
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
    
    V3DD::CLEAR_DRAWING("ugv_nav4d_start_aabb");
    V3DD::DRAW_WIREFRAME_BOX("ugv_nav4d_start_aabb", start.position + Eigen::Vector3d(0, 0, travConfig.distToGround),
                       start.orientation, base::Vector3d(travConfig.robotSizeX, travConfig.robotSizeY, travConfig.robotHeight), V3DD::Color::cyan);
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
    std::thread t([this](){
        V3DD::CONFIGURE_DEBUG_DRAWINGS_USE_EXISTING_WIDGET(this->widget);

        this->plan(this->start, this->goal);
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




