#include <iostream>
#include <boost/archive/polymorphic_binary_iarchive.hpp>
#include <motion_planning_libraries/sbpl/SbplMotionPrimitives.hpp>
#include <motion_planning_libraries/sbpl/SbplSplineMotionPrimitives.hpp>
#include <fstream>
#include <backward/backward.hpp>
#include <envire_core/items/Item.hpp>
#include <envire_core/graph/EnvireGraph.hpp>

#include "Planner.hpp"
#include "PlannerGui.h"

#include <base/geometry/Spline.hpp>
#include <iostream>
#include <fstream>

// backward::SignalHandling sh;

using namespace ::maps::grid;
using namespace base::geometry;

int main(int argc, char** argv)
{
    PlannerGui gui(argc, argv);
    gui.exec();
    return 0;
    
//     if(argc < 2)
//     {
//         std::cout << "Usage 'cmd mlsFileName' forward_turn_multiplier backward_turn_multiplier later_multiplier " << std::endl;
//         return 0;
//     }
// 
//     std::ifstream fileIn(argv[1]);
// 
//     MLSMapPrecalculated mlsSloped;
//     {
//         // deserialize from string stream
//         envire::core::EnvireGraph g;
//         g.loadFromFile(argv[1]);
//         maps::grid::MLSMapKalman mlsInput = (*g.getItem<envire::core::Item<maps::grid::MLSMapKalman>>("mls_map", 0)).getData();
// //         MLSMapSloped mlsInput;
// //         mlsIn >> mlsInput;
//         mlsSloped = MLSMapPrecalculated(mlsInput);
//     }
//     std::cout << "MLS Resolutition " << mlsSloped.getResolution().transpose() << std::endl;
//     assert(mlsSloped.getResolution().x() == mlsSloped.getResolution().y());
//     
//     motion_planning_libraries::SplinePrimitivesConfig config;
//     config.gridSize = mlsSloped.getResolution().x();
//     config.destinationCircleRadius = 6;
//     config.numAngles = 16;
//     config.numEndAngles = 8;
//     config.cellSkipFactor = 0.1;
//     config.generatePointTurnMotions = false;
//     config.generateBackwardMotions = true;
//     
//     motion_planning_libraries::Mobility mobility;
//     mobility.mSpeed = 1.3;
//     mobility.mTurningSpeed = 0.4;
//     mobility.mMinTurningRadius = 0.1;
//     
//     mobility.mMultiplierForward = 1;
//     mobility.mMultiplierBackward = 1;
//     mobility.mMultiplierLateral = 5;
//     mobility.mMultiplierBackwardTurn = 2;
//     mobility.mMultiplierForwardTurn = 1;
//     mobility.mMultiplierPointTurn = 8; 
//     
//     ugv_nav4d::TraversabilityConfig conf;
//     conf.gridResolution =  mlsSloped.getResolution().x();
//     conf.maxSlope = 70.0/180.0 * M_PI;
//     conf.maxStepHeight = 0.5; //space below robot
//     conf.robotSizeX = 0.5;
//     conf.robotSizeY =  0.7;
//     conf.robotHeight = 0.9; //incl space below body
//     
//     
//     switch(argc)
//     {
//         case 0:
//         case 1:
//             std::cout << "Usage 'cmd mlsFileName' forward_turn_multiplier backward_turn_multiplier later_multiplier " << std::endl;
//             return 0;
//         case 5:
//             mobility.mMultiplierLateral = atof(argv[4]);
//         case 4:
//             mobility.mMultiplierBackwardTurn = atof(argv[3]);
//         case 3:
//             mobility.mMultiplierForwardTurn = atof(argv[2]);
//             break;
//     }
//     
//     
//     ugv_nav4d::Planner planner(config, conf, mobility);
//     
//     base::samples::RigidBodyState start;
//     start.position << 0,-0,-0.7;//garage
//     start.orientation.setIdentity();
//     base::samples::RigidBodyState end;
//     end.position << 4, 5, 3.23207; //garage
//     end.orientation.setIdentity();
//     
//     planner.updateMap(mlsSloped);
//     std::cout << "planning" << std::endl;
//     if(!planner.plan(base::Time::fromMilliseconds(16000), start, end))
//     {
//         std::cout << "PLANNER FAIL" << std::endl;
// //         return 0;
//     }
//     std::vector<base::Trajectory> path;
//     std::cout << "PLANNING DONE" << std::endl;
//     planner.getTrajectory(path);
//     std::cout << "GOT TRAJECTORY" << std::endl;
//     
// 
// 
//     viz.updateTr(path);
// 
//     mlsViz.updateData(mlsSloped);
//     mlsViz.setPluginEnabled(false);
//     mlsViz.setCycleHeightColor(true);
//     mlsViz.setShowPatchExtents(false);
//     mlsViz.setShowNormals(true);
// 
//     trav3dViz.updateData(planner.getTraversabilityMap());
// //     primViz.updateData(planner.getEnv()->getAvailableMotions().getPrimitives());
//     envViz.setGridSize(config.gridSize);
//     envViz.setSolutionMotions(planner.getMotions());
//     envViz.setStartPos(start.position.x(), start.position.y(), start.position.z());
//     envViz.updateData(*planner.getEnv().get());
//     motion_planning_libraries::SbplSplineMotionPrimitives primitives(config);
//     splineViz.updateData(primitives);
// //     
//      widget.show();
// // 
// //     
// // 
// //     
//      app.exec();
//     return 0;
}
