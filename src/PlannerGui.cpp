#include "PlannerGui.h"
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>
#include <QFileDialog>
#include <thread>
#include <motion_planning_libraries/Config.hpp>
#include "Planner.hpp"


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
    
    loadMls();
    
    QVBoxLayout* layout = new QVBoxLayout();
       
    layout->addWidget(&widget);

    
    slopeMetricSpinBox = new QDoubleSpinBox();
    slopeMetricSpinBox->setMinimum(0);
    slopeMetricSpinBox->setMaximum(9999999999999);
    slopeMetricSpinBox->setValue(1);
    layout->addWidget(slopeMetricSpinBox);
    
    window.setLayout(layout);


    //to be able to send Trajectory via slot
    qRegisterMetaType<std::vector<ugv_nav4d::Motion>>("std::vector<ugv_nav4d::Motion>");
    qRegisterMetaType<std::vector<base::Trajectory>>("std::vector<base::Trajectory>");
    qRegisterMetaType<maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase*>>("maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase*>");

    connect(&mlsViz, SIGNAL(picked(float,float,float)), this, SLOT(picked(float,float,float)));
    connect(&trav3dViz, SIGNAL(picked(float,float,float)), this, SLOT(picked(float,float,float)));
    
    connect(this, SIGNAL(plannerDone(std::vector<base::Trajectory>,maps::grid::TraversabilityMap3d<maps::grid::TraversabilityNodeBase*>,std::vector<ugv_nav4d::Motion>)),
            this, SLOT(setPlannerResult(std::vector<base::Trajectory>,maps::grid::TraversabilityMap3d<maps::grid::TraversabilityNodeBase*>,std::vector<ugv_nav4d::Motion>)));
    
    connect(slopeMetricSpinBox, SIGNAL(editingFinished()), this, SLOT(slopeMetricEditingFinished()));
    
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
        try
        {
            boost::archive::binary_iarchive mlsIn(fileIn);
            maps::grid::MLSMapSloped mlsInput;
            mlsIn >> mlsInput;
            mlsMap = maps::grid::MLSMapPrecalculated(mlsInput);
            mlsViz.updateData(mlsMap);
            return;
        }
        catch(...) {}
        
        //NOTE try/catch does not work for boost deserialization. It will just segault
        //if the file is broken or in another format.
        //A solution might be to try deserialization in a fork and see if the subprocess crashes.
        
/*        try
        {
            boost::archive::binary_iarchive mlsIn(fileIn);
            envire::core::EnvireGraph g;
            g.loadFromFile(file.toStdString());
            maps::grid::MLSMapKalman mlsInput = (*g.getItem<envire::core::Item<maps::grid::MLSMapKalman>>("mls_map", 0)).getData();
            mlsMap = maps::grid::MLSMapPrecalculated(mlsInput);
            mlsViz.updateData(mlsMap);
            return;
        }
        catch(...) {}   */   
        
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


void PlannerGui::slopeMetricEditingFinished()
{
    if(start.allFinite() && goal.allFinite())
        startPlanThread();   
}


void PlannerGui::startPlanThread()
{
    std::thread t([this](){
        this->plan(this->start, this->goal, this->slopeMetricSpinBox->value());
    });
    t.detach(); //needed to avoid destruction of thread at end of method
}


void PlannerGui::setPlannerResult(const std::vector<base::Trajectory>& path,
                                  const maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase*>& travMap,
                                  const std::vector<ugv_nav4d::Motion> motions)
{
    trajViz.updateTr(path);
    trav3dViz.updateData(travMap);
    
    envViz.setGridSize(mlsMap.getResolution().x());
    envViz.setSolutionMotions(motions);
    envViz.setStartPos(start.x(), start.y(), start.z());
}

void PlannerGui::plan(const Eigen::Vector3f& start, const Eigen::Vector3f& goal,
                      const double slopeMetricScale)
{
    motion_planning_libraries::SplinePrimitivesConfig config;
    config.gridSize = 0.2;// mlsMap.getResolution().x();
    config.destinationCircleRadius = 8;
    config.numAngles = 10;
    config.numEndAngles = 5;
    config.cellSkipFactor = 0.1;
    config.generatePointTurnMotions = false;
    
    motion_planning_libraries::Mobility mobility;
    mobility.mSpeed = 1.3;
    mobility.mTurningSpeed = 0.4;
    mobility.mMinTurningRadius = 0.1;
    
    mobility.mMultiplierForward = 1;
    mobility.mMultiplierBackward = 1;
    mobility.mMultiplierLateral = 5;
    mobility.mMultiplierBackwardTurn = 2;
    mobility.mMultiplierForwardTurn = 1;
    mobility.mMultiplierPointTurn = 8;
    
    ugv_nav4d::TraversabilityConfig conf;
    conf.gridResolution = 0.2;//  mlsMap.getResolution().x();
    conf.maxSlope = 30.0/180.0 * M_PI;
    conf.maxStepHeight = 0.5; //space below robot
    conf.robotSizeX = 0.5;
    conf.robotSizeY =  0.7;
    conf.robotHeight = 0.9; //incl space below body
    conf.slopeMetricScale = slopeMetricScale;
    
    ugv_nav4d::Planner planner(config, conf, mobility);
    
    base::samples::RigidBodyState startState;
    startState.position = start.cast<double>();
    startState.orientation.setIdentity();
    base::samples::RigidBodyState endState;
    endState.position << goal.cast<double>();
    endState.orientation.setIdentity();
    
    planner.updateMap(mlsMap);
    
    std::cout << std::endl << std::endl;
    std::cout << "Planning: " << start.transpose() << " -> " << goal.transpose() << std::endl;
    const bool result = planner.plan(base::Time::fromSeconds(60), startState, endState);

    if(!result)
        std::cout << "FAIL" << std::endl;
    
    std::vector<base::Trajectory> path;
    if(result)
    {
        planner.getTrajectory(path);
        std::cout << "DONE" << std::endl;
    }
    
    envViz.setHeuristic(planner.getEnv()->debugCost);
    envViz.setCollisionPoses(planner.getEnv()->debugCollisionPoses);
    envViz.setRobotHalfSize(planner.getEnv()->robotHalfSize);
    emit plannerDone(path, planner.getTraversabilityMap(), planner.getMotions());
}

