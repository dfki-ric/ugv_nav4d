#include "PlannerDump.hpp"
#include "Planner.hpp"
#define WRITE(X) output.write(reinterpret_cast<const char*>(&X), sizeof X)
#define READ(X)  input.read(reinterpret_cast<char*>(&X), sizeof X)
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/filesystem/operations.hpp>
#include <fstream>
#include <base-logging/Logging.hpp>

ugv_nav4d::PlannerDump::PlannerDump(const std::string& dumpName)
{
    LOG_INFO_S << "Loading Dump " << dumpName;
    
    std::ifstream input(dumpName, std::ios::binary | std::ios::in);
    
    READ(traversabilityConfig);
    READ(mobility);
    READ(splinePrimitiveConfig);
    READ(plannerConfig);
    base::Pose tmp;
    READ(tmp);
    start.setPose(tmp);
    READ(tmp);
    goal.setPose(tmp);
    double maxTimed;
    READ(maxTimed);
    
    boost::archive::binary_iarchive ia(input);
    ia >> travMap;
}

ugv_nav4d::PlannerDump::PlannerDump(const ugv_nav4d::Planner& planner, 
                                    const std::string& filePostfix, 
                                    const base::Time& maxTimeA, 
                                    const base::samples::RigidBodyState& startbody2Mls, 
                                    const base::samples::RigidBodyState& endbody2Mls)
{
    const std::string targetFile = getUnusedFilename(filePostfix);
    LOG_INFO_S << "Dumping planner state to: " << targetFile;
    std::ofstream output(targetFile, std::ios::binary | std::ios::out|std::ios::trunc);

    WRITE(planner.traversabilityConfig);
    WRITE(planner.mobility);
    WRITE(planner.splinePrimitiveConfig);
    WRITE(planner.plannerConfig);
    base::Pose tmp = startbody2Mls.getPose();
    WRITE(tmp);
    tmp = endbody2Mls.getPose();
    WRITE(tmp);
    double maxTimeD = maxTimeA.toSeconds();
    
    WRITE(maxTimeD);
    
    boost::archive::binary_oarchive oa(output);
    oa << *(planner.env->getTraversabilityMap());
    output.flush();
    output.close();
}

std::string ugv_nav4d::PlannerDump::getUnusedFilename(const std::string& filePostfix) const
{
    std::string completeName;
    bool firstTime = true;
    do{        
        completeName = "ugv4d_dump_" + base::Time::now().toString(base::Time::Seconds, "%Y-%m-%d_%H%M%S") + "_" + filePostfix + ".bin";

        if(!firstTime)
            usleep(10000);
        
        firstTime = false;
    } while(boost::filesystem::exists(completeName));
    
    return completeName;
}


// void Planner::createDump(const std::string& filePostfix, const base::Time& maxTime, const base::samples::RigidBodyState& startbody2Mls, const base::samples::RigidBodyState& endbody2Mls) const
// {
//     std::ofstream output(getUnusedFilename(filePostfix), std::ios::binary | std::ios::out|std::ios::trunc);
// 
//     WRITE(traversabilityConfig);
//     WRITE(mobility);
//     WRITE(splinePrimitiveConfig);
//     base::Pose tmp = startbody2Mls.getPose();
//     WRITE(tmp);
//     tmp = endbody2Mls.getPose();
//     WRITE(tmp);
//     double maxTimeD = maxTime.toSeconds();
//     
//     WRITE(maxTimeD);
//     
//     boost::archive::binary_oarchive oa(output);
//     oa << env->getMlsMap();
//     output.flush();
//     output.close();
// }
// 
// Planner::PLANNING_RESULT Planner::plan(const std::string& dumpName, std::vector< base::Trajectory >& resultTrajectory)
// {
//     std::ifstream input(dumpName, std::ios::binary | std::ios::in);
//     
//     READ(traversabilityConfig);
//     READ(const_cast<sbpl_spline_primitives::Mobility &>(mobility));
//     READ(const_cast<sbpl_spline_primitives::SplinePrimitivesConfig &>(splinePrimitiveConfig));
//     base::Pose start;
//     READ(start);
//     base::Pose goal;
//     READ(goal);
//     double maxTime;
//     READ(maxTime);
//     
//     boost::shared_ptr<MLSBase> mlsPtr(new MLSBase());
//     boost::archive::binary_iarchive ia(input);
//     ia >> *(mlsPtr.get());
// 
//     //resetup environment
//     env.reset(new EnvironmentXYZTheta(mlsPtr, traversabilityConfig, splinePrimitiveConfig, mobility));
// 
//     base::samples::RigidBodyState startRBS;
//     startRBS.setPose(start);
//     base::samples::RigidBodyState goalRBS;
//     goalRBS.setPose(goal);
//     
//     return plan(base::Time::fromSeconds(maxTime), startRBS, goalRBS, resultTrajectory);
// }
