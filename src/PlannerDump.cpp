#include "PlannerDump.hpp"
#include "Planner.hpp"
#define WRITE(X) output.write(reinterpret_cast<const char*>(&X), sizeof X)
#define READ(X)  input.read(reinterpret_cast<char*>(&X), sizeof X)
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/filesystem/operations.hpp>

ugv_nav4d::PlannerDump::PlannerDump(const std::string& dumpName)
{
    std::cout << "Loading Dump " << dumpName << std::endl;
    
    std::ifstream input(dumpName, std::ios::binary | std::ios::in);
    
    READ(traversabilityConfig);
    READ(mobility);
    READ(splinePrimitiveConfig);
    base::Pose tmp;
    READ(tmp);
    start.setPose(tmp);
    READ(tmp);
    goal.setPose(tmp);
    double maxTimed;
    READ(maxTimed);

    boost::archive::binary_iarchive ia(input);
    ia >> mlsMap;
}

ugv_nav4d::PlannerDump::PlannerDump(const ugv_nav4d::Planner& planner, const std::string& filePostfix, const base::Time& maxTimeA, const base::samples::RigidBodyState& startbody2Mls, const base::samples::RigidBodyState& endbody2Mls)
{
    std::ofstream output(getUnusedFilename(filePostfix), std::ios::binary | std::ios::out|std::ios::trunc);

    WRITE(planner.traversabilityConfig);
    WRITE(planner.mobility);
    WRITE(planner.splinePrimitiveConfig);
    base::Pose tmp = startbody2Mls.getPose();
    WRITE(tmp);
    tmp = endbody2Mls.getPose();
    WRITE(tmp);
    double maxTimeD = maxTimeA.toSeconds();
    
    WRITE(maxTimeD);
    
    boost::archive::binary_oarchive oa(output);
    oa << planner.env->getMlsMap();
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
//     READ(const_cast<motion_planning_libraries::Mobility &>(mobility));
//     READ(const_cast<motion_planning_libraries::SplinePrimitivesConfig &>(splinePrimitiveConfig));
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
