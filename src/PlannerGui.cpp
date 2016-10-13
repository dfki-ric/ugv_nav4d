#include "PlannerGui.h"
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>
#include <QFileDialog>
#include <thread>
#include <motion_planning_libraries/Config.hpp>
#include "Planner.hpp"
#include "PreComputedMotions.hpp"


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
    
    
    mlsViz.setCycleHeightColor(true);
    mlsViz.setShowPatchExtents(false);
    mlsViz.setShowNormals(false);
    
    trajViz.setLineWidth(5);
    trajViz.setColor(QColor("Cyan"));
        
    QVBoxLayout* layout = new QVBoxLayout();
       
    layout->addWidget(&widget);    
    window.setLayout(layout);

    //to be able to send Trajectory via slot
    qRegisterMetaType<std::vector<ugv_nav4d::Motion>>("std::vector<ugv_nav4d::Motion>");
    qRegisterMetaType<std::vector<base::Trajectory>>("std::vector<base::Trajectory>");
    qRegisterMetaType<maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase*>>("maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase*>");

    connect(&mlsViz, SIGNAL(picked(float,float,float)), this, SLOT(picked(float,float,float)));
    connect(&trav3dViz, SIGNAL(picked(float,float,float)), this, SLOT(picked(float,float,float)));
    
    connect(this, SIGNAL(plannerDone()), this, SLOT(plannerIsDone()));
    
    config.gridSize = 0.1;// mlsMap.getResolution().x();
    config.destinationCircleRadius = 5;
    config.numAngles = 10;
    config.numEndAngles = 5;
    config.cellSkipFactor = 0.01;
    config.generatePointTurnMotions = false;
    
    mobility.mSpeed = 2.3;
    mobility.mTurningSpeed = 3.4;
    mobility.mMinTurningRadius = 0.08;
    
    mobility.mMultiplierForward = 1;
    mobility.mMultiplierBackward = 1;
    mobility.mMultiplierLateral = 2;
    mobility.mMultiplierBackwardTurn = 2;
    mobility.mMultiplierForwardTurn = 1;
    mobility.mMultiplierPointTurn = 8;
     
    conf.gridResolution = 0.1;//  mlsMap.getResolution().x();
    conf.maxSlope = 40.0/180.0 * M_PI;
    conf.maxStepHeight = 0.5; //space below robot
    conf.robotSizeX = 0.5;
    conf.robotSizeY =  0.7;
    conf.robotHeight = 0.9; //incl space below body
    conf.slopeMetricScale = 0.0;
    
    planner.reset(new ugv_nav4d::Planner(config, conf, mobility));
    
    motion_planning_libraries::SbplSplineMotionPrimitives primitives(config);
    splineViz.setMaxCurvature(ugv_nav4d::PreComputedMotions::calculateCurvatureFromRadius(mobility.mMinTurningRadius));
    splineViz.updateData(primitives);
    
    loadMls();
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
        std::ifstream fileIn(file.toStdString());       
//         try
//         {
//             boost::archive::binary_iarchive mlsIn(fileIn);
//             maps::grid::MLSMapSloped mlsInput;
//             mlsIn >> mlsInput;
//             mlsMap = maps::grid::MLSMapPrecalculated(mlsInput);
//             mlsViz.updateData(mlsMap);
//             planner->updateMap(mlsMap);
//             return;
//         }
//         catch(...) {}
        
        try
        {
            boost::archive::binary_iarchive mlsIn(fileIn);
            envire::core::EnvireGraph g;
            g.loadFromFile(file.toStdString());
            maps::grid::MLSMapKalman mlsInput = (*g.getItem<envire::core::Item<maps::grid::MLSMapKalman>>("mls_map", 0)).getData();
            mlsMap = maps::grid::MLSMapPrecalculated(mlsInput);
            mlsViz.updateData(mlsMap);
            planner->updateMap(mlsMap);
            return;
        }
        catch(...) {}   
        

        
        std::cerr << "Unabled to load mls. Unknown format" << std::endl;
    }
}


void PlannerGui::picked(float x, float y, float z)
{   
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
        startPlanThread();
        pickStart = true;
    }
}


void PlannerGui::startPlanThread()
{
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
    envViz.setStartPos(start.x(), start.y(), start.z());
    
    envViz.setHeuristic(planner->getEnv()->debugHeuristic);
    envViz.setCollisionPoses(planner->getEnv()->debugCollisionPoses);
    envViz.setRobotHalfSize(planner->getEnv()->robotHalfSize);
    envViz.setSuccessors(planner->getEnv()->debugSuccessors);
}

void PlannerGui::plan(const Eigen::Vector3f& start, const Eigen::Vector3f& goal)
{
    planner->getEnv()->debugSuccessors.clear();
    planner->getEnv()->debugCollisionPoses.clear();
    
    base::samples::RigidBodyState startState;
    startState.position = start.cast<double>();
    startState.orientation.setIdentity();
    base::samples::RigidBodyState endState;
    endState.position << goal.cast<double>();
    endState.orientation.setIdentity();
    
    std::cout << std::endl << std::endl;
    std::cout << "Planning: " << start.transpose() << " -> " << goal.transpose() << std::endl;
    const bool result = planner->plan(base::Time::fromSeconds(500), startState, endState);
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

