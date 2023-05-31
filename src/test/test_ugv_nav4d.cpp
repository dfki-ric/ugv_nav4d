#include <fstream>
#include <cstdlib>

#include "gtest/gtest.h"

#include "ugv_nav4d/DiscreteTheta.hpp"
#include "ugv_nav4d/Planner.hpp"
#include <traversability_generator3d/TraversabilityConfig.hpp>
#include <maps/grid/MLSMap.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

using namespace ugv_nav4d;

class PlannerTest : public testing::Test {
protected:

  void SetUp() override;
  void TearDown() override;
  void loadMlsMap(const std::string& path);
  std::string getResult(const Planner::PLANNING_RESULT& result);

  Planner* planner;
  maps::grid::MLSMapSloped mlsMap;
  PlannerConfig plannerConfig;
  Mobility mobility;
  sbpl_spline_primitives::SplinePrimitivesConfig splinePrimitiveConfig;
  traversability_generator3d::TraversabilityConfig traversabilityConfig;
  bool map_loaded = false;
};

void PlannerTest::SetUp(){

  std::cout << "Called SetUp()" << std::endl;

  splinePrimitiveConfig.gridSize=0.3;
  splinePrimitiveConfig.numAngles=42;
  splinePrimitiveConfig.numEndAngles=21;
  splinePrimitiveConfig.destinationCircleRadius=10;
  splinePrimitiveConfig.cellSkipFactor=3;
  splinePrimitiveConfig.splineOrder=4.0;

  mobility.translationSpeed=0.5;
  mobility.rotationSpeed=0.5;
  mobility.minTurningRadius=1;
  mobility.spline_sampling_resolution=0.05;
  mobility.remove_goal_offset=true;
  mobility.multiplierForward=1;
  mobility.multiplierBackward=3;
  mobility.multiplierPointTurn=3;
  mobility.multiplierLateral=4;
  mobility.multiplierForwardTurn=2;
  mobility.multiplierBackwardTurn=4;
  mobility.multiplierLateralCurve=4;
  mobility.searchRadius=0.0;
  mobility.searchProgressSteps=0.1;
  mobility.maxMotionCurveLength=100;

  traversabilityConfig.maxStepHeight=0.25;
  traversabilityConfig.maxSlope=0.45;
  traversabilityConfig.inclineLimittingMinSlope=0.2;
  traversabilityConfig.inclineLimittingLimit=0.1;
  traversabilityConfig.usePathStatistics=false;
  traversabilityConfig.costFunctionDist=0.0;
  traversabilityConfig.minTraversablePercentage=0.4;
  traversabilityConfig.robotHeight=1.2;
  traversabilityConfig.robotSizeX=1.35;
  traversabilityConfig.robotSizeY=0.85;
  traversabilityConfig.distToGround=0.0;
  traversabilityConfig.slopeMetricScale=1.0;
  traversabilityConfig.slopeMetric=traversability_generator3d::NONE;
  traversabilityConfig.gridResolution=0.3;
  traversabilityConfig.initialPatchVariance=0.0001;
  traversabilityConfig.allowForwardDownhill=true;
  traversabilityConfig.enableInclineLimitting=false;

  plannerConfig.searchUntilFirstSolution=false;
  plannerConfig.initialEpsilon=64;
  plannerConfig.epsilonSteps=2;
  plannerConfig.numThreads=8;
  const std::string path = std::string(getenv("AUTOPROJ_CURRENT_ROOT")) + "/planning/ugv_nav4d/test_data/Plane1Mio.ply";
  std::cout << "Path to PLY: \n: " << path << "\n";
  loadMlsMap(path);

}

void PlannerTest::TearDown(){
  std::cout << "Called TearDown()" << std::endl;
  delete planner;
}

void PlannerTest::loadMlsMap(const std::string& path){
  std::ifstream fileIn(path);
  if(path.find(".ply") != std::string::npos)
  {
      std::cout << "Loading PLY \n";
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

          const double mls_res = 0.3;
          const double size_x = ma.x;
          const double size_y = ma.y;

          const maps::grid::Vector2ui numCells(size_x / mls_res + 1, size_y / mls_res + 1);
          std::cout << "NUM CELLS: " << numCells << "\n";
          maps::grid::MLSConfig cfg;
          cfg.gapSize = 0.1;
          cfg.thickness = 0.1;
          cfg.useColor = false;
          mlsMap = maps::grid::MLSMapSloped(numCells, maps::grid::Vector2d(mls_res, mls_res), cfg);
          mlsMap.mergePointCloud(*cloud, base::Transform3d::Identity());
          map_loaded = true;
          return;
      }
  }
  std::cout << "No .ply file provided! \n";
  map_loaded = false;
}

std::string PlannerTest::getResult(const Planner::PLANNING_RESULT& result){

  std::string result_str;

  switch(result)
  {
      case Planner::GOAL_INVALID:
          result_str = "GOAL_INVALID";
          break;
      case Planner::START_INVALID:
          result_str = "START_INVALID";
          break;
      case Planner::NO_SOLUTION:
          result_str = "NO_SOLUTION";
          break;
     case Planner::NO_MAP:
          result_str = "NO_MAP";
          break;
      case Planner::INTERNAL_ERROR:
          result_str = "INTERNAL_ERROR";
          break;
      case Planner::FOUND_SOLUTION:
          result_str = "FOUND_SOLUTION";
          break;
      default:
          result_str = "ERROR unknown result state";
          break;
  }
  return result_str;
}


TEST_F(PlannerTest, check_planner_init_failure_wrong_grid_resolutions) {
  traversabilityConfig.gridResolution=0.4;

  try {
    planner = new Planner(splinePrimitiveConfig,
                          traversabilityConfig,
                          mobility,
                          plannerConfig);
  }
  catch(const std::exception& ex){
    std::cout << "Exception: \n" << ex.what() << "\n";
    EXPECT_NE(splinePrimitiveConfig.gridSize, traversabilityConfig.gridResolution);
  }
}

TEST_F(PlannerTest, check_planner_init_success) {
  planner = new Planner(splinePrimitiveConfig,
                        traversabilityConfig,
                        mobility,
                        plannerConfig);

  EXPECT_NE(planner, nullptr);
}

TEST_F(PlannerTest, check_planner_goal_invalid) {

  EXPECT_EQ(map_loaded, true);

  planner = new Planner(splinePrimitiveConfig,
                        traversabilityConfig,
                        mobility,
                        plannerConfig);

  EXPECT_NE(planner, nullptr);
  planner->updateMap(mlsMap);

  base::samples::RigidBodyState startState;
  startState.position.x() = 2.3;
  startState.position.y() = 1.2;
  startState.position.z() = 0.0;

  startState.orientation.w() = 1;
  startState.orientation.x() = 0;
  startState.orientation.y() = 0;
  startState.orientation.z() = 0;

  base::samples::RigidBodyState endState;
  endState.position.x() = 11.0;
  endState.position.y() = 0.0;
  endState.position.z() = 0.0;

  endState.orientation.w() = 1;
  endState.orientation.x() = 0;
  endState.orientation.y() = 0;
  endState.orientation.z() = 0;

  int maxTime = 5;

  std::vector<trajectory_follower::SubTrajectory> trajectory2D;
  std::vector<trajectory_follower::SubTrajectory> trajectory3D;

  std::cout << "Planning: \n";
  const Planner::PLANNING_RESULT result = planner->plan(base::Time::fromSeconds(maxTime),
                                          startState, endState, trajectory2D, trajectory3D);
  std::cout << "Planning Result: " << getResult(result) << std::endl;
  EXPECT_EQ(result, Planner::GOAL_INVALID);
}

TEST_F(PlannerTest, check_planner_start_invalid) {

  EXPECT_EQ(map_loaded, true);

  planner = new Planner(splinePrimitiveConfig,
                        traversabilityConfig,
                        mobility,
                        plannerConfig);

  EXPECT_NE(planner, nullptr);
  planner->updateMap(mlsMap);

  base::samples::RigidBodyState startState;
  startState.position.x() = 0.0;
  startState.position.y() = 0.0;
  startState.position.z() = 0.0;

  startState.orientation.w() = 1;
  startState.orientation.x() = 0;
  startState.orientation.y() = 0;
  startState.orientation.z() = 0;

  base::samples::RigidBodyState endState;
  endState.position.x() = 1.0;
  endState.position.y() = 0.0;
  endState.position.z() = 0.0;

  endState.orientation.w() = 1;
  endState.orientation.x() = 0;
  endState.orientation.y() = 0;
  endState.orientation.z() = 0;

  int maxTime = 5;

  std::vector<trajectory_follower::SubTrajectory> trajectory2D;
  std::vector<trajectory_follower::SubTrajectory> trajectory3D;

  std::cout << "Planning: \n";
  const Planner::PLANNING_RESULT result = planner->plan(base::Time::fromSeconds(maxTime),
                                          startState, endState, trajectory2D, trajectory3D);
  std::cout << "Planning Result: " << getResult(result) << std::endl;
  EXPECT_EQ(result, Planner::START_INVALID);
}

TEST_F(PlannerTest, check_planner_success) {

  EXPECT_EQ(map_loaded, true);

  planner = new Planner(splinePrimitiveConfig,
                        traversabilityConfig,
                        mobility,
                        plannerConfig);

  EXPECT_NE(planner, nullptr);
  planner->updateMap(mlsMap);

  base::samples::RigidBodyState startState;
  startState.position.x() = 2.3;
  startState.position.y() = 4.1;
  startState.position.z() = 0.0;

  startState.orientation.w() = 1;
  startState.orientation.x() = 0;
  startState.orientation.y() = 0;
  startState.orientation.z() = 0;

  base::samples::RigidBodyState endState;
  endState.position.x() = 6.1;
  endState.position.y() = 4.2;
  endState.position.z() = 0.0;

  endState.orientation.w() = 1;
  endState.orientation.x() = 0;
  endState.orientation.y() = 0;
  endState.orientation.z() = 0;

  int maxTime = 5;

  std::vector<trajectory_follower::SubTrajectory> trajectory2D;
  std::vector<trajectory_follower::SubTrajectory> trajectory3D;

  std::cout << "Planning: \n";
  const Planner::PLANNING_RESULT result = planner->plan(base::Time::fromSeconds(maxTime),
                                          startState, endState, trajectory2D, trajectory3D);
  std::cout << "Planning Result: " << getResult(result) << std::endl;
  EXPECT_EQ(result, Planner::FOUND_SOLUTION);
}

//DiscreteTheta.hpp
TEST(UGV_NAV4D_TEST, check_discrete_theta_init) {
  DiscreteTheta theta = DiscreteTheta(0,16);
  EXPECT_NEAR(theta.getRadian(),0, 0.001);

  theta = DiscreteTheta(1,16);
  EXPECT_EQ(theta.getNumAngles(),16);
  EXPECT_NEAR(theta.getRadian(),0.3926, 0.001);

  theta = DiscreteTheta(-1,16);
  EXPECT_EQ(theta.getTheta(),15);
  EXPECT_NEAR(theta.getRadian(),5.8904, 0.001);

  theta = DiscreteTheta(18,16);
  EXPECT_EQ(theta.getTheta(),2);

  theta = DiscreteTheta(16,16);
  EXPECT_EQ(theta.getTheta(),0);

  theta = DiscreteTheta(5.90,16);
  EXPECT_EQ(theta.getTheta(),15);

  theta = DiscreteTheta(5.45,16);
  EXPECT_NEAR(theta.getRadian(),5.45, 0.1);
  EXPECT_EQ(theta.getTheta(),14);

  DiscreteTheta thetaA = DiscreteTheta(3,16);
  DiscreteTheta thetaB = DiscreteTheta(5,16);
  EXPECT_EQ(thetaA.shortestDist(thetaB).getTheta(),2);

  thetaA = DiscreteTheta(2,16);
  thetaB = DiscreteTheta(14,16);
  EXPECT_EQ(thetaA.shortestDist(thetaB).getTheta(),4);
}

int main(int argc, char ** argv){
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
