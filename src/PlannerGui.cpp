#include "PlannerGui.h"
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>
#include <QFileDialog>
#include <QPushButton>
#include <thread>
#include <motion_planning_libraries/Config.hpp>
#include "Planner.hpp"
#include "PreComputedMotions.hpp"
#include <vizkit3dDebugDrawings/DrawingManager.h>
#include "UgvDebug.hpp"


PlannerGui::PlannerGui(int argc, char** argv): QObject()
{
    start.orientation.setIdentity();
    goal.orientation.setIdentity();
    
    widget = vizkit3dDebugDrawings::DrawingManager::instance()->getVizkit3DWidget();
    
    widget->setCameraManipulator(vizkit3d::ORBIT_MANIPULATOR);
    widget->addPlugin(&splineViz);
    widget->addPlugin(&trajViz);
    widget->addPlugin(&mlsViz);
    widget->addPlugin(&trav3dViz);
    widget->addPlugin(&envViz);
    widget->addPlugin(&startViz);
    widget->addPlugin(&goalViz);
    
    splineViz.setPluginEnabled(false);
    
    mlsViz.setCycleHeightColor(true);
    mlsViz.setShowPatchExtents(false);
    mlsViz.setShowNormals(false);
    
    trajViz.setLineWidth(5);
    trajViz.setColor(QColor("Cyan"));
        
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
    time->setMaximum(999);
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
    
    heuristicComboBox = new QComboBox();
    heuristicComboBox->addItem("HEURISTIC_2D");
    heuristicComboBox->addItem("HEURISTIC_3D");

    connect(heuristicComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(heuristicComboBoxIndexChanged(int)));
    QLabel* heuristicComboLabel = new QLabel();
    slopeMetricComboLabel->setText("Heuristic Type");
    QHBoxLayout* heuristicTypeLayout = new QHBoxLayout();
    heuristicTypeLayout->addWidget(heuristicComboLabel);
    heuristicTypeLayout->addWidget(heuristicComboBox);
    layout->addLayout(heuristicTypeLayout);
    
    
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
    
    parallelismCheckBox = new QCheckBox();
    parallelismCheckBox->setChecked(false);
    QLabel* parallelismLabel = new QLabel();
    parallelismLabel->setText("Parallel getSuccs()");
    
    QHBoxLayout* parallelismLayout = new QHBoxLayout();
    parallelismLayout->addWidget(parallelismLabel);
    parallelismLayout->addWidget(parallelismCheckBox);
    
    layout->addLayout(parallelismLayout);
    connect(parallelismCheckBox, SIGNAL(stateChanged(int)), this, SLOT(parallelismCheckBoxStateChanged(int)));
    
    
    QPushButton* replanButton = new QPushButton("Replan");
    timeLayout->addWidget(replanButton);
    
    connect(replanButton, SIGNAL(released()), this, SLOT(replanButtonReleased()));
    connect(expandButton, SIGNAL(released()), this, SLOT(expandPressed()));
    
    layout->addWidget(bar);
    
    window.setLayout(layout);

    //to be able to send Trajectory via slot
    qRegisterMetaType<std::vector<ugv_nav4d::Motion>>("std::vector<ugv_nav4d::Motion>");
    qRegisterMetaType<std::vector<base::Trajectory>>("std::vector<base::Trajectory>");
    qRegisterMetaType<maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase*>>("maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase*>");

    connect(&mlsViz, SIGNAL(picked(float,float,float)), this, SLOT(picked(float,float,float)));
    connect(&trav3dViz, SIGNAL(picked(float,float,float)), this, SLOT(picked(float,float,float)));
    connect(this, SIGNAL(plannerDone()), this, SLOT(plannerIsDone()));
    
    double res = 0.1;
     if(argc > 2)
         res = atof(argv[2]);
    
    std::cout << "RES = " << res << std::endl;
    config.gridSize = res;
    config.destinationCircleRadius = 5;
    config.numAngles = 24;
    config.numEndAngles = 12;
    config.cellSkipFactor = 1.0;
    config.generatePointTurnMotions = false;
    config.generateLateralMotions = false;
    config.splineOrder = 4;
    
    mobility.mSpeed = 1.3;
    mobility.mTurningSpeed = 5.4;
    mobility.mMinTurningRadius = 0.2;
    
    mobility.mMultiplierForward = 1;
    mobility.mMultiplierBackward = 1;
    mobility.mMultiplierLateral = 2;
    mobility.mMultiplierBackwardTurn = 2;
    mobility.mMultiplierForwardTurn = 1;
    mobility.mMultiplierPointTurn = 8;
     
    conf.gridResolution = res;
    conf.maxSlope = 40.0/180.0 * M_PI;
    maxSlopeSpinBox->setValue(40);
    conf.maxStepHeight = 0.5; //space below robot
    conf.robotSizeX = 0.5;
    conf.robotSizeY =  0.7;
    conf.robotHeight = 0.9; //incl space below body
    conf.slopeMetricScale = 0.0;
    conf.slopeMetric = SlopeMetric::NONE;
    conf.heuristicType = HeuristicType::HEURISTIC_2D;
    conf.inclineLimittingMinSlope = 10.0 * M_PI/180.0;
    conf.inclineLimittingLimit = 5.0 * M_PI/180.0;
    conf.parallelismEnabled = false;
    
    inclineLimittingMinSlopeSpinBox->setValue(10);
    inclineLimittingLimitSpinBox->setValue(6);
    
    planner.reset(new ugv_nav4d::Planner(config, conf, mobility));
    
    motion_planning_libraries::SbplSplineMotionPrimitives primitives(config);
    splineViz.setMaxCurvature(ugv_nav4d::PreComputedMotions::calculateCurvatureFromRadius(mobility.mMinTurningRadius));
    
    
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
    
    if(path.find("graph_mls_kalman") != std::string::npos)
    {
        try
        {
            envire::core::EnvireGraph g;
            g.loadFromFile(path);
            maps::grid::MLSMapKalman mlsInput = (*g.getItem<envire::core::Item<maps::grid::MLSMapKalman>>("mls_map", 0)).getData();
            mlsMap = maps::grid::MLSMapPrecalculated(mlsInput);
            mlsViz.updateMLSPrecalculated(mlsMap);
            planner->updateMap(mlsMap);
            return;
        }
        catch(...) {}   
    }
    else
    {
        try
        {
            boost::archive::binary_iarchive mlsIn(fileIn);
            maps::grid::MLSMapSloped mlsInput;
            mlsIn >> mlsInput;
            mlsMap = maps::grid::MLSMapPrecalculated(mlsInput);
            mlsViz.updateMLSPrecalculated(mlsMap);
            planner->updateMap(mlsMap);
            return;
        }
        catch(...) {}
    }
    
    

    
    std::cerr << "Unabled to load mls. Unknown format" << std::endl;
    
}


void PlannerGui::picked(float x, float y, float z)
{   
//     start << 5.91327,  1.38306, -1.39575;
//     goal <<  7.47328,  1.34183, -1.39437;
//     startPlanThread();
    expandButton->setEnabled(true);
    if(pickStart)
    {
        start.position << x, y, z;
        QVector3D pos(start.position.x(), start.position.y(), start.position.z());
        startViz.setTranslation(pos);
        std::cout << "Start: " << start.position.transpose() << std::endl;
        pickStart = false;
    }
    else
    {
        goal.position << x, y, z;
        QVector3D pos(goal.position.x(), goal.position.y(), goal.position.z());
        goalViz.setTranslation(pos);
        std::cout << "goal: " << goal.position.transpose() << std::endl;
        planner->setTravConfig(conf);
        startPlanThread();
        pickStart = true;
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
    if(index < metrics.size())
    {
        conf.slopeMetric = metrics[index];
    }
    else
    {
        throw std::runtime_error("unknown slope index");
    }
}

void PlannerGui::heuristicComboBoxIndexChanged(int index)
{
    std::vector<HeuristicType> heuristics = {HeuristicType::HEURISTIC_2D, HeuristicType::HEURISTIC_3D};
    if(index < heuristics.size())
    {
        conf.heuristicType = heuristics[index];
    }
    else
    {
        throw std::runtime_error("unknown heuristic index");
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
        this->plan(this->start, this->goal);
    });
    t.detach(); //needed to avoid destruction of thread at end of method
}


void PlannerGui::plannerIsDone()
{
    std::vector<base::Trajectory> path;
    planner->getTrajectory(path);    
    
    double pathLen = 0;
    for(const base::Trajectory& traj : path)
    {
        pathLen += traj.spline.length(traj.spline.getStartParam(), traj.spline.getEndParam(), traj.spline.getGeometricResolution());
    }
    
    std::cout << "path length:  " << pathLen << std::endl;
    
    trajViz.updateTr(path);
    trajViz.setLineWidth(8);
    
    trav3dViz.updateData((planner->getEnv()->getTraversabilityBaseMap()));
    
    envViz.setGridSize(mlsMap.getResolution().x());   
    envViz.setRobotHalfSize(planner->getEnv()->robotHalfSize);
    
    UGV_DEBUG(
        envViz.setEnvDebugData(planner->getEnv()->debugData);
        envViz.setTravGenDebugData(planner->getEnv()->getTravGen().debugData);
    )

    bar->setMaximum(1);
}

void PlannerGui::expandPressed()
{
    planner->getEnv()->getTravGen().clearTrMap();
    planner->getEnv()->getTravGen().setConfig(conf);
    planner->getEnv()->getTravGen().expandAll(start.position.cast<double>());
    trav3dViz.updateData((planner->getEnv()->getTraversabilityBaseMap()));
    mlsViz.setPluginEnabled(false);
}


/* 
Start:  5.99972 0.399847 -1.31341d
goal: -0.455198   7.99133   2.08586
*/

void PlannerGui::plan(const base::Pose& start, const base::Pose& goal)
{
    UGV_DEBUG(
        planner->getEnv()->debugData.getSuccs().clear();
        planner->getEnv()->debugData.getCollisions().clear();
        planner->getEnv()->debugData.getSlopeData().clear();
        planner->getEnv()->debugData.getSlopeCandidates().clear();
    )
    
    base::samples::RigidBodyState startState;
    startState.position = start.position;
    startState.orientation = start.orientation;
    base::samples::RigidBodyState endState;
    endState.position << goal.position;
    endState.orientation = goal.orientation;
    
    std::cout << std::endl << std::endl;
    std::cout << "Planning: " << start << " -> " << goal << std::endl;
    const bool result = planner->plan(base::Time::fromSeconds(time->value()), startState, endState);
    if(result)
    {
        std::cout << "DONE" << std::endl;
    }
    else
    {
        std::cout << "FAIL" << std::endl;
    }

    emit plannerDone();
}




