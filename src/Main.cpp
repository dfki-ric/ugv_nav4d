#include <iostream>
#include <boost/archive/polymorphic_binary_iarchive.hpp>
#include <motion_planning_libraries/sbpl/SbplMotionPrimitives.hpp>
#include <fstream>
#include <backward/backward.hpp>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/TrajectoryVisualization.hpp>
#include <vizkit3d/MLSMapVisualization.hpp>
#include <vizkit3d/TraversabilityMap3dVisualization.hpp>
#include <vizkit3d/EnvironmentXYZThetaVisualization.hpp>

#include "Planner.hpp"

// backward::SignalHandling sh;

using namespace ::maps::grid;

int main(int argc, char** argv)
{
        if(argc < 2)
    {
        std::cout << "Usage 'cmd mlsFileName' " << std::endl;
        return 0;
    }
    
    std::ifstream fileIn(argv[1]);

    // deserialize from string stream
    boost::archive::polymorphic_binary_iarchive mlsIn(fileIn);
    MLSMapSloped mlsSloped;
    
    mlsIn >> mlsSloped;
    
    std::cout << "MLS Resolutition " << mlsSloped.getResolution().transpose() << std::endl;
    
    
    //create motion primitives
    motion_planning_libraries::MotionPrimitivesConfig config;
    config.mMobility.mSpeed = 1.3;
    config.mMobility.mTurningSpeed = 0.4;
    config.mMobility.mMinTurningRadius = 0.1;
    
    //FIXME what do the multipliers do?
    config.mMobility.mMultiplierForward = 1;
    config.mMobility.mMultiplierBackward = 5;
    config.mMobility.mMultiplierLateral = 10;
    config.mMobility.mMultiplierBackwardTurn = 2;
    config.mMobility.mMultiplierForwardTurn = 2;
    config.mMobility.mMultiplierPointTurn = 8;
    
    config.mNumPrimPartition = 8;
    config.mNumPosesPerPrim = 10;
    config.mNumAngles = 16;
    
    config.mMapWidth = 400; //FIXME why do I need this when generating primitives?
    config.mMapHeight = 400;
    config.mGridSize = 0.1;
    config.mPrimAccuracy = 0.1;
    
    
    TraversabilityGenerator3d::Config conf;
    conf.gridResolution = 0.1;
    conf.maxSlope = 0.5;
    conf.maxStepHeight = 0.2; //space below robot
    conf.robotSizeX = 0.5;
    conf.robotHeight = 0.9; //incl space below body
    
    Planner planner(config, conf);
    
    base::samples::RigidBodyState start;
    start.position = Eigen::Vector3d(0,-0,-0.7);
    start.orientation.setIdentity();
    base::samples::RigidBodyState end;
    end.position = Eigen::Vector3d(4, 5, 3.23207);
    end.orientation.setIdentity();
    
    planner.updateMap(mlsSloped);
    if(!planner.plan(base::Time::fromSeconds(2), start, end))
        return 0;

    std::vector<base::Trajectory> path;

    planner.getTrajectory(path);
    

    QApplication app(argc, argv);
    vizkit3d::Vizkit3DWidget widget;
    widget.setCameraManipulator(vizkit3d::ORBIT_MANIPULATOR);

    vizkit3d::TrajectoryVisualization viz;
    vizkit3d::MLSMapVisualization mlsViz;
    vizkit3d::TraversabilityMap3dVisualization trav3dViz;
    vizkit3d::EnvironmentXYZThetaVisualization envViz;
    widget.addPlugin(&viz);
    widget.addPlugin(&mlsViz);
    widget.addPlugin(&trav3dViz);
    widget.addPlugin(&envViz);
    viz.setLineWidth(10);
    viz.updateTr(path);
    mlsViz.updateData(mlsSloped);
    trav3dViz.updateData(planner.getTraversabilityMap());
    
    envViz.updateData(*planner.getEnv().get());
    
    widget.show();
    

    
    app.exec();
    return 0;
}
