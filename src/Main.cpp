#include <iostream>
#include <boost/archive/polymorphic_binary_iarchive.hpp>
#include <motion_planning_libraries/sbpl/SbplMotionPrimitives.hpp>
#include "EnvironmentXYZTheta.hpp"
#include <sbpl/planners/araplanner.h>
#include <fstream>
#include <backward/backward.hpp>
#include <sbpl/utils/mdpconfig.h>

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
    
    MultiLevelGridMap<SurfacePatchBase> *mlsBase = new MultiLevelGridMap<SurfacePatchBase>(mlsSloped.getNumCells(), mlsSloped.getResolution(), mlsSloped.getLocalMapData());

    std::cout << "MLSBase Resolutition " << mlsBase->getResolution().transpose() << std::endl;
    
    for(size_t y = 0 ; y < mlsSloped.getNumCells().y(); y++)
    {
        for(size_t x = 0 ; x < mlsSloped.getNumCells().x(); x++)
        {
            Index idx(x, y);
            for(auto &p : mlsSloped.at(idx))
            {
//                 std::cout << "Patch ! " << idx.transpose() << std::endl;
                SurfacePatchBase basePatch(p.getTop(), p.getTop() - p.getBottom());
                mlsBase->at(idx).insert(basePatch);
            }
        }
    }
    
    boost::shared_ptr<MultiLevelGridMap<SurfacePatchBase>> mlsPtr(mlsBase);
    
    std::cout << "MLS Size " << mlsBase->getSize().transpose() << std::endl;
    
    //create motion primitives
    motion_planning_libraries::MotionPrimitivesConfig config;
    config.mSpeeds.mSpeedForward = 1.0;
    config.mSpeeds.mSpeedBackward = 1.0;
    config.mSpeeds.mSpeedLateral = 0.3;
    config.mSpeeds.mSpeedTurn = 0.4;
    config.mSpeeds.mSpeedPointTurn = 0.1;
    
    //FIXME what do the multipliers do?
    config.mSpeeds.mMultiplierForward = 1;
    config.mSpeeds.mMultiplierBackward = 5;
    config.mSpeeds.mMultiplierLateral = 10;
    config.mSpeeds.mMultiplierTurn = 2;
    config.mSpeeds.mMultiplierPointTurn = 8;
    
    config.mNumPrimPartition = 10;
    config.mNumPosesPerPrim = 30;
    config.mNumAngles = 16;
    
    config.mMapWidth = 400; //FIXME why do I need this when generating primitives?
    config.mMapHeight = 400;
    config.mGridSize = 0.1;
    
    motion_planning_libraries::SbplMotionPrimitives mprims(config);
    mprims.createPrimitives();
    
    TraversabilityGenerator3d::Config conf;
    conf.gridResolution = 0.1;
    conf.maxSlope = 0.5;
    conf.maxStepHeight = 0.2;
    conf.robotSizeX = 0.5;
    conf.robotHeight = 0.9;

    
    EnvironmentXYZTheta myEnv(mlsPtr, conf, mprims);
    
//     anaPlanner planner(&myEnv, true);

    ARAPlanner planner(&myEnv, true);
    
    planner.set_search_mode(true);
    
    myEnv.setStart(Eigen::Vector3d(0, 0,-0.7), 0);
    myEnv.setGoal(Eigen::Vector3d(4, 0,-0.7), 0);

    MDPConfig mdp_cfg;
        
    if (! myEnv.InitializeMDPCfg(&mdp_cfg)) {
        std::cout << "InitializeMDPCfg failed, start and goal id cannot be requested yet" << std::endl;
        return false;
    }
        
    std::cout << "SBPL: About to set start and goal, startid" << mdp_cfg.startstateid << std::endl;
    if (planner.set_start(mdp_cfg.startstateid) == 0) {
        std::cout << "Failed to set start state" << std::endl;
        return false;
    }

    if (planner.set_goal(mdp_cfg.goalstateid) == 0) {
        std::cout << "Failed to set goal state" << std::endl;
        return false;
    }

    

    
    std::vector<int> solution;
    
    planner.replan(10.0, &solution);
    std::cout << "Solution: " << std::endl;
    for(const int i : solution)
      myEnv.PrintState(i, true);
    std::cout << std::endl;

    
    std::vector<PlannerStats> stats;
    
    planner.get_search_stats(&stats);
    
    std::cout << std::endl << "Stats" << std::endl;
    for(const PlannerStats &s: stats)
    {
        std::cout << "cost " << s.cost << " time " << s.time << "num childs " << s.expands << std::endl;
    }
    
//     FILE *debug = fopen("debug.txt", "w");
    
//     planner.print_searchpath(nullptr);
    
    return 0;
}
