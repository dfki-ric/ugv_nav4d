#include "PlannerGui.h"
#include <QFileDialog>
#include <QPushButton>
#include <thread>
#include <ugv_nav4d/Config.hpp>
#include <ugv_nav4d/Planner.hpp>
#include <ugv_nav4d/PreComputedMotions.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>
#include <vizkit3d_debug_drawings/DebugDrawingColors.hpp>
#include <boost/filesystem.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <ugv_nav4d/PlannerDump.hpp>
#include <pcl/common/transforms.h>

using namespace ugv_nav4d;

PlannerGui::PlannerGui(const std::string& dumpName): QObject()
{
    setupUI();
    
    PlannerDump dump(dumpName);

    mobility = dump.getMobilityConf();
    conf = dump.getTravConfig();
    config = dump.getSplineConfig();
    plannerConf = dump.getPlannerConfig();
    
    planner.reset(new ugv_nav4d::Planner(config, conf, mobility, plannerConf));
    
    sbpl_spline_primitives::SbplSplineMotionPrimitives primitives(config);
    splineViz.setMaxCurvature(ugv_nav4d::PreComputedMotions::calculateCurvatureFromRadius(mobility.minTurningRadius));
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
    
    splineViz.setPluginEnabled(false);
    
    mlsViz.setCycleHeightColor(true);
    mlsViz.setShowPatchExtents(false); 
    mlsViz.setShowNormals(false);
    
    trajViz.setLineWidth(5);
    trajViz.setColor(QColor("Cyan"));
    trajViz2.setLineWidth(5);
    trajViz2.setColor(QColor("magenta"));
        
    QVBoxLayout* layout = new QVBoxLayout();
    
       
    layout->addWidget(widget);
    
    expandButton = new QPushButton("Create travMap");
    expandButton->setEnabled(false);
    maxSlopeSpinBox = new QDoubleSpinBox();
    maxSlopeSpinBox->setMinimum(1);
    maxSlopeSpinBox->setMaximum(60);
    connect(maxSlopeSpinBox, SIGNAL(editingFinished()), this, SLOT(maxSlopeEditingFinished()));
    QHBoxLayout* slopeLayout = new QHBoxLayout();
    QLabel* lab = new QLabel();
    lab->setText("max slope (deg):");
    slopeLayout->addWidget(lab);
    slopeLayout->addWidget(maxSlopeSpinBox);
    slopeLayout->addWidget(expandButton);
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
    
    
    
    parallelismCheckBox = new QCheckBox();
    parallelismCheckBox->setChecked(false);
    QLabel* parallelismLabel = new QLabel();
    parallelismLabel->setText("Parallel getSuccs()");
    
    QHBoxLayout* parallelismLayout = new QHBoxLayout();
    parallelismLayout->addWidget(parallelismLabel);
    parallelismLayout->addWidget(parallelismCheckBox);
    
    layout->addLayout(parallelismLayout);
    connect(parallelismCheckBox, SIGNAL(stateChanged(int)), this, SLOT(parallelismCheckBoxStateChanged(int)));
    
    
    QPushButton* replanButton = new QPushButton("Plan");
    timeLayout->addWidget(replanButton);
    
    QPushButton* dumpButton = new QPushButton("Create PlannerDump");
    timeLayout->addWidget(dumpButton);

    
    connect(replanButton, SIGNAL(released()), this, SLOT(replanButtonReleased()));
    connect(expandButton, SIGNAL(released()), this, SLOT(expandPressed()));
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
    
    config.gridSize = res;
    config.numAngles = 24;
    config.numEndAngles = 12;
    config.destinationCircleRadius = 5;
    config.cellSkipFactor = 1.0;
    config.generatePointTurnMotions = true;
    config.generateLateralMotions = false;
    config.generateBackwardMotions = true;
    config.splineOrder = 4;
    
    mobility.translationSpeed = 0.2;
    mobility.rotationSpeed = 0.6;
    mobility.minTurningRadius = 0.2; // increase this to reduce the number of available motion primitives
    
    mobility.multiplierForward = 1;
    mobility.multiplierBackward = 1;
    mobility.multiplierLateral = 3;
    mobility.multiplierBackwardTurn = 1;
    mobility.multiplierForwardTurn = 1;
    mobility.multiplierPointTurn = 3;
     
    conf.gridResolution = res;
    conf.maxSlope = 0.57; //40.0/180.0 * M_PI;
    conf.maxStepHeight = 0.3; //space below robot
    conf.robotSizeX = 0.9;
    conf.robotSizeY =  0.5;
    conf.robotHeight = 0.9; //incl space below body
    conf.slopeMetricScale = 0.0;
    conf.slopeMetric = SlopeMetric::NONE;
    conf.inclineLimittingMinSlope = 0.22; // 10.0 * M_PI/180.0;
    conf.inclineLimittingLimit = 0.43;// 5.0 * M_PI/180.0;
    conf.parallelismEnabled = false;
    conf.costFunctionDist = 0.4;
    conf.distToGround = 0.2;
    conf.minTraversablePercentage = 0.5;
    conf.allowForwardDownhill = true;
    plannerConf.epsilonSteps = 2.0;
    plannerConf.initialEpsilon = 20.0;
    
    planner.reset(new ugv_nav4d::Planner(config, conf, mobility, plannerConf));
    
    sbpl_spline_primitives::SbplSplineMotionPrimitives primitives(config);
    
    splineViz.setMaxCurvature(ugv_nav4d::PreComputedMotions::calculateCurvatureFromRadius(mobility.minTurningRadius));
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

        
        std::cout << "Start: " << start.position.transpose() << std::endl;
        pickStart = false;
        expandButton->setEnabled(true);
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
        std::cout << "Loading PLY" << std::endl;
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
            std::cout << "MIN: " << mi << ", MAX: " << ma << std::endl;
        
            const double mls_res = conf.gridResolution;
            const double size_x = ma.x;
            const double size_y = ma.y;
            
            const maps::grid::Vector2ui numCells(size_x / mls_res + 1, size_y / mls_res + 1);
            std::cout << "NUM CELLS: " << numCells << std::endl;
            
            maps::grid::MLSConfig cfg;
            cfg.gapSize = 0.1;
            mlsMap = maps::grid::MLSMapKalman(numCells, maps::grid::Vector2d(mls_res, mls_res), cfg);
            mlsMap.mergePointCloud(*cloud, base::Transform3d::Identity());
            mlsViz.updateMLSKalman(mlsMap);
            planner->updateMap(mlsMap);
        }
        return;
    }

    
    
    try
    {
        std::cout << "Loading MLS" << std::endl;
        boost::archive::binary_iarchive mlsIn(fileIn);
        mlsIn >> mlsMap;
        mlsViz.updateMLSKalman(mlsMap);
        planner->updateMap(mlsMap);
        return;
    }
    catch(...) {}
    

    std::cerr << "Unabled to load mls. Unknown format" << std::endl;
    
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
            start.position.z() += conf.distToGround; //because we click on the ground but need to put robot position
            
            V3DD::CLEAR_DRAWING("ugv_nav4d_start_aabb");
            V3DD::DRAW_WIREFRAME_BOX("ugv_nav4d_start_aabb", start.position +  base::Vector3d(0, 0, conf.distToGround / 2.0), start.orientation,
                               base::Vector3d(conf.robotSizeX, conf.robotSizeY, conf.robotHeight - conf.distToGround), V3DD::Color::cyan);
            
            QVector3D pos(start.position.x(), start.position.y(), start.position.z());
            startViz.setTranslation(pos);
            std::cout << "Start: " << start.position.transpose() << std::endl;
            startPicked = true;
            expandButton->setEnabled(true);
        }
            break;
        case 4: //right click
        {
            goal.position << x, y, z;
            goal.position.z() += conf.distToGround;
            QVector3D pos(goal.position.x(), goal.position.y(), goal.position.z());
            goalViz.setTranslation(pos);
            std::cout << "goal: " << goal.position.transpose() << std::endl;
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
    conf.maxSlope = maxSlopeSpinBox->value()/180.0 * M_PI;
}

void PlannerGui::inclineLimittingLimitSpinBoxEditingFinished()
{
    conf.inclineLimittingLimit = inclineLimittingLimitSpinBox->value()/180.0 * M_PI;
}

void PlannerGui::inclineLimittingMinSlopeSpinBoxEditingFinished()
{
    conf.inclineLimittingMinSlope = inclineLimittingMinSlopeSpinBox->value()/180.0 * M_PI;
}

void PlannerGui::slopeMetricScaleSpinBoxEditingFinished()
{
    conf.slopeMetricScale = slopeMetricScaleSpinBox->value();
}

void PlannerGui::slopeMetricComboBoxIndexChanged(int index)
{
    std::vector<SlopeMetric> metrics = {SlopeMetric::NONE, SlopeMetric::AVG_SLOPE, SlopeMetric::MAX_SLOPE,
                                        SlopeMetric::TRIANGLE_SLOPE};
    if(size_t(index) < metrics.size())
    {
        conf.slopeMetric = metrics[index];
    }
    else
    {
        throw std::runtime_error("unknown slope index");
    }
}


void PlannerGui::parallelismCheckBoxStateChanged(int)
{
    conf.parallelismEnabled = parallelismCheckBox->isChecked();
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
    V3DD::DRAW_WIREFRAME_BOX("ugv_nav4d_start_aabb", start.position + Eigen::Vector3d(0, 0, conf.distToGround),
                       start.orientation, base::Vector3d(conf.robotSizeX, conf.robotSizeY, conf.robotHeight), V3DD::Color::cyan);
}

void PlannerGui::obstacleDistanceSpinBoxEditingFinished()
{
    conf.costFunctionDist = obstacleDistanceSpinBox->value();
}

void PlannerGui::obstacleFactorSpinBoxEditingFinished()
{
    throw std::runtime_error("Function removed");
//     conf.costFunctionObstacleMultiplier = obstacleFactorSpinBox->value();
}

void PlannerGui::timeEditingFinished()
{
    
}

void PlannerGui::replanButtonReleased()
{
    planner->setTravConfig(conf);
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
    trajViz.updateTr(path);
    trajViz.setLineWidth(8);

    trajViz2.updateTr(beautifiedPath);
    trajViz2.setLineWidth(8);    
    
    
    
    trav3dViz.updateData((planner->getEnv()->getTraversabilityMap().copyCast<maps::grid::TraversabilityNodeBase *>()));
    obstacleMapViz.updateData((planner->getEnv()->getObstacleMap().copyCast<maps::grid::TraversabilityNodeBase *>()));
    
    bar->setMaximum(1);
}

void PlannerGui::expandPressed()
{
    planner->getEnv()->getTravGen().clearTrMap();
    planner->getEnv()->getTravGen().setConfig(conf);
    //expand position needs to be on map
    planner->getEnv()->getTravGen().expandAll(start.position - base::Position(0, 0, conf.distToGround));
    
    planner->getEnv()->getObstacleGen().clearTrMap();
    planner->getEnv()->getObstacleGen().setConfig(conf);
    planner->getEnv()->getObstacleGen().expandAll(start.position - base::Position(0, 0, conf.distToGround));
    
    trav3dViz.updateData((planner->getEnv()->getTraversabilityMap().copyCast<maps::grid::TraversabilityNodeBase *>()));
    obstacleMapViz.updateData((planner->getEnv()->getObstacleMap().copyCast<maps::grid::TraversabilityNodeBase *>()));
    mlsViz.setPluginEnabled(false);
}

void PlannerGui::dumpPressed()
{
    std::cout << "Dumping" << std::endl;
    
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
    
    std::cout << std::endl << std::endl;
    std::cout << "Planning: " << start << " -> " << goal << std::endl;
    const Planner::PLANNING_RESULT result = planner->plan(base::Time::fromSeconds(time->value()),
                                            startState, endState, path, beautifiedPath);
    
    switch(result)
    {
        case Planner::GOAL_INVALID:
            std::cout << "GOAL_INVALID" << std::endl;
            break;
        case Planner::START_INVALID:
            std::cout << "START_INVALID" << std::endl;
            break; 
        case Planner::NO_SOLUTION:
            std::cout << "NO_SOLUTION" << std::endl;
            break;
       case Planner::NO_MAP:
            std::cout << "NO_MAP" << std::endl;
            break;
        case Planner::INTERNAL_ERROR:
            std::cout << "INTERNAL_ERROR" << std::endl;
            break;
        case Planner::FOUND_SOLUTION:
            std::cout << "FOUND_SOLUTION" << std::endl;
            break;
        default:
            std::cout << "ERROR unknown result state" << std::endl;
            break;
    }
    
    emit plannerDone();
}




