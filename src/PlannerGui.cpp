#include "PlannerGui.h"
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>
#include <QFileDialog>
#include <QPushButton>
#include <thread>
#include <motion_planning_libraries/Config.hpp>
#include "Planner.hpp"
#include "PreComputedMotions.hpp"

#include "UgvDebug.hpp"


PlannerGui::PlannerGui(int argc, char** argv): QObject(), app(argc, argv)
{
    
    
    start.setConstant(std::numeric_limits<float>::infinity());
    goal.setConstant(std::numeric_limits<float>::infinity());
    
    widget.setCameraManipulator(vizkit3d::ORBIT_MANIPULATOR);
    widget.addPlugin(&splineViz);
    widget.addPlugin(&trajViz);
    widget.addPlugin(&mlsViz);
    widget.addPlugin(&trav3dViz);
    widget.addPlugin(&envViz);
    
    splineViz.setPluginEnabled(false);
    
    mlsViz.setCycleHeightColor(true);
    mlsViz.setShowPatchExtents(false);
    mlsViz.setShowNormals(false);
    
    trajViz.setLineWidth(5);
    trajViz.setColor(QColor("Cyan"));
        
    QVBoxLayout* layout = new QVBoxLayout();
    
       
    layout->addWidget(&widget);
    
    expandButton = new QPushButton("Create travMap");
    expandButton->setEnabled(false);
    maxSlopeSpinBox = new QDoubleSpinBox();
    maxSlopeSpinBox->setMinimum(1);
    maxSlopeSpinBox->setMaximum(60);
    connect(maxSlopeSpinBox, SIGNAL(editingFinished()), this, SLOT(maxSlopeEditingFinished()));
    QHBoxLayout* slopeLayout = new QHBoxLayout();
    QLabel* lab = new QLabel();
    lab->setText("max slope:");
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
    lab2->setText("Max planning time:");
    timeLayout->addWidget(lab2);
    timeLayout->addWidget(time);
    timeLayout->addWidget(time);
    layout->addLayout(timeLayout);
    connect(time, SIGNAL(editingFinished()), this, SLOT(timeEditingFinished()));
    
    bar = new QProgressBar();
    bar->setMinimum(0);
    bar->setMaximum(1);
//     bar->hide();
    
    
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
         res =atof(argv[2]);
    
    std::cout << "RES = " << res << std::endl;
    config.gridSize = res;
    config.destinationCircleRadius = 12;
    config.numAngles = 16;
    config.numEndAngles = 7;
    config.cellSkipFactor = 0.1;
    config.generatePointTurnMotions = false;
    
    mobility.mSpeed = 1.3;
    mobility.mTurningSpeed = 5.4;
    mobility.mMinTurningRadius = 0.1;
    
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
    conf.inclineLimittingMinSlope = 10.0 * M_PI/180.0;
    conf.inclineLimittingLimit = 5.0 * M_PI/180.0;
    
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

void PlannerGui::exec()
{
    window.show();
    app.exec();
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
            mlsViz.updateData(mlsMap);
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
            mlsViz.updateData(mlsMap);
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
        start << x, y, z;
        std::cout << "Start: " << start.transpose() << std::endl;
        pickStart = false;
    }
    else
    {
        goal << x, y, z;
        std::cout << "goal: " << goal.transpose() << std::endl;
        planner->setTravConfig(conf);
        startPlanThread();
        pickStart = true;
    }
}


void PlannerGui::maxSlopeEditingFinished()
{
    conf.maxSlope = maxSlopeSpinBox->value()/180.0 * M_PI;
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
    planner->getEnv()->getTravGen().expandAll(start.cast<double>());
    trav3dViz.updateData((planner->getEnv()->getTraversabilityBaseMap()));
    mlsViz.setPluginEnabled(false);
}


/* 
Start:  5.99972 0.399847 -1.31341d
goal: -0.455198   7.99133   2.08586
*/

void PlannerGui::plan(const Eigen::Vector3f& start, const Eigen::Vector3f& goal)
{
    UGV_DEBUG(
        planner->getEnv()->debugData.getSuccs().clear();
        planner->getEnv()->debugData.getCollisions().clear();
    )
    
    base::samples::RigidBodyState startState;
    startState.position = start.cast<double>();
    startState.orientation.setIdentity();
    base::samples::RigidBodyState endState;
    endState.position << goal.cast<double>();
    endState.orientation.setIdentity();
    
    std::cout << std::endl << std::endl;
    std::cout << "Planning: " << start.transpose() << " -> " << goal.transpose() << std::endl;
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

