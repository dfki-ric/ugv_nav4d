#define BOOST_TEST_MODULE PlannerTestModule
#include <boost/test/included/unit_test.hpp>

#include <fstream>
#include <cstdlib>

#include "ugv_nav4d/DiscreteTheta.hpp"
#include "ugv_nav4d/Planner.hpp"
#include <traversability_generator3d/TraversabilityConfig.hpp>
#include <maps/grid/MLSMap.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

using namespace ugv_nav4d;

std::string filePath;

class PlannerTest {
public:
    PlannerTest();
    ~PlannerTest();
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

PlannerTest::PlannerTest() {
    splinePrimitiveConfig.gridSize = 0.3;
    splinePrimitiveConfig.numAngles = 42;
    splinePrimitiveConfig.numEndAngles = 21;
    splinePrimitiveConfig.destinationCircleRadius = 10;
    splinePrimitiveConfig.cellSkipFactor = 3;
    splinePrimitiveConfig.splineOrder = 4.0;

    mobility.translationSpeed = 0.5;
    mobility.rotationSpeed = 0.5;
    mobility.minTurningRadius = 1;
    mobility.spline_sampling_resolution = 0.05;
    mobility.remove_goal_offset = true;
    mobility.multiplierForward = 1;
    mobility.multiplierBackward = 3;
    mobility.multiplierPointTurn = 3;
    mobility.multiplierLateral = 4;
    mobility.multiplierForwardTurn = 2;
    mobility.multiplierBackwardTurn = 4;
    mobility.multiplierLateralCurve = 4;
    mobility.searchRadius = 0.0;
    mobility.searchProgressSteps = 0.1;
    mobility.maxMotionCurveLength = 100;

    traversabilityConfig.maxStepHeight = 0.25;
    traversabilityConfig.maxSlope = 0.45;
    traversabilityConfig.inclineLimittingMinSlope = 0.2;
    traversabilityConfig.inclineLimittingLimit = 0.1;
    traversabilityConfig.costFunctionDist = 0.0;
    traversabilityConfig.minTraversablePercentage = 0.4;
    traversabilityConfig.robotHeight = 1.2;
    traversabilityConfig.robotSizeX = 1.35;
    traversabilityConfig.robotSizeY = 0.85;
    traversabilityConfig.distToGround = 0.0;
    traversabilityConfig.slopeMetricScale = 1.0;
    traversabilityConfig.slopeMetric = traversability_generator3d::NONE;
    traversabilityConfig.gridResolution = 0.3;
    traversabilityConfig.initialPatchVariance = 0.0001;
    traversabilityConfig.allowForwardDownhill = true;
    traversabilityConfig.enableInclineLimitting = false;

    plannerConfig.searchUntilFirstSolution = false;
    plannerConfig.initialEpsilon = 64;
    plannerConfig.epsilonSteps = 2;
    plannerConfig.numThreads = 8;

    loadMlsMap(filePath);
}

PlannerTest::~PlannerTest() {
    delete planner;
}

void PlannerTest::loadMlsMap(const std::string& path) {
    std::ifstream fileIn(path);
    if (path.find(".ply") != std::string::npos) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PLYReader plyReader;
        if (plyReader.read(path, *cloud) >= 0) {
            pcl::PointXYZ mi, ma;
            pcl::getMinMax3D(*cloud, mi, ma);

            Eigen::Affine3f pclTf = Eigen::Affine3f::Identity();
            pclTf.translation() << -mi.x, -mi.y, -mi.z;
            pcl::transformPointCloud(*cloud, *cloud, pclTf);

            const double mls_res = 0.3;
            const double size_x = ma.x;
            const double size_y = ma.y;

            const maps::grid::Vector2ui numCells(size_x / mls_res + 1, size_y / mls_res + 1);
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
    map_loaded = false;
}

std::string PlannerTest::getResult(const Planner::PLANNING_RESULT& result) {
    switch (result) {
        case Planner::GOAL_INVALID: return "GOAL_INVALID";
        case Planner::START_INVALID: return "START_INVALID";
        case Planner::NO_SOLUTION: return "NO_SOLUTION";
        case Planner::NO_MAP: return "NO_MAP";
        case Planner::INTERNAL_ERROR: return "INTERNAL_ERROR";
        case Planner::FOUND_SOLUTION: return "FOUND_SOLUTION";
        default: return "ERROR unknown result state";
    }
}

// Individual Tests
BOOST_FIXTURE_TEST_SUITE(PlannerTestSuite, PlannerTest)

// Converted test case from GTest to Boost
BOOST_AUTO_TEST_CASE(check_planner_init_failure_wrong_grid_resolutions) {
    traversabilityConfig.gridResolution = 0.4;

    try {
        planner = new Planner(splinePrimitiveConfig, traversabilityConfig, mobility, plannerConfig);
    } catch (const std::exception& ex) {
        std::cout << "Exception: \n" << ex.what() << "\n";
        BOOST_CHECK_NE(splinePrimitiveConfig.gridSize, traversabilityConfig.gridResolution);
    }
}

BOOST_AUTO_TEST_CASE(check_planner_init_success) {
    planner = new Planner(splinePrimitiveConfig, traversabilityConfig, mobility, plannerConfig);
    BOOST_CHECK(planner != nullptr);
}

BOOST_AUTO_TEST_CASE(check_planner_goal_invalid) {
    BOOST_REQUIRE(map_loaded); // equivalent to EXPECT_EQ(map_loaded, true)

    planner = new Planner(splinePrimitiveConfig, traversabilityConfig, mobility, plannerConfig);
    BOOST_REQUIRE(planner != nullptr);
    planner->updateMap(mlsMap);

    base::samples::RigidBodyState startState;
    startState.position.x() = 2.3;
    startState.position.y() = 1.2;
    startState.position.z() = 0.0;
    startState.orientation.w() = 1;

    base::samples::RigidBodyState endState;
    endState.position.x() = 11.0;
    endState.position.y() = 0.0;
    endState.position.z() = 0.0;
    endState.orientation.w() = 1;

    int maxTime = 5;
    std::vector<trajectory_follower::SubTrajectory> trajectory2D;
    std::vector<trajectory_follower::SubTrajectory> trajectory3D;

    std::cout << "Planning: \n";
    const Planner::PLANNING_RESULT result = planner->plan(base::Time::fromSeconds(maxTime), startState, endState, trajectory2D, trajectory3D);
    std::cout << "Planning Result: " << getResult(result) << std::endl;
    BOOST_CHECK_EQUAL(result, Planner::GOAL_INVALID);
}

BOOST_AUTO_TEST_CASE(check_planner_start_invalid) {
    BOOST_REQUIRE(map_loaded);

    planner = new Planner(splinePrimitiveConfig, traversabilityConfig, mobility, plannerConfig);
    BOOST_REQUIRE(planner != nullptr);
    planner->updateMap(mlsMap);

    base::samples::RigidBodyState startState;
    startState.position.x() = 0.0;
    startState.position.y() = 0.0;
    startState.position.z() = 0.0;
    startState.orientation.w() = 1;

    base::samples::RigidBodyState endState;
    endState.position.x() = 1.0;
    endState.position.y() = 0.0;
    endState.position.z() = 0.0;
    endState.orientation.w() = 1;

    int maxTime = 5;
    std::vector<trajectory_follower::SubTrajectory> trajectory2D;
    std::vector<trajectory_follower::SubTrajectory> trajectory3D;

    std::cout << "Planning: \n";
    const Planner::PLANNING_RESULT result = planner->plan(base::Time::fromSeconds(maxTime), startState, endState, trajectory2D, trajectory3D);
    std::cout << "Planning Result: " << getResult(result) << std::endl;
    BOOST_CHECK_EQUAL(result, Planner::START_INVALID);
}

BOOST_AUTO_TEST_CASE(check_planner_success) {
    BOOST_REQUIRE(map_loaded);

    planner = new Planner(splinePrimitiveConfig, traversabilityConfig, mobility, plannerConfig);
    BOOST_REQUIRE(planner != nullptr);
    planner->updateMap(mlsMap);

    base::samples::RigidBodyState startState;
    startState.position.x() = 2.3;
    startState.position.y() = 4.1;
    startState.position.z() = 0.0;
    startState.orientation.w() = 1;

    base::samples::RigidBodyState endState;
    endState.position.x() = 6.1;
    endState.position.y() = 4.2;
    endState.position.z() = 0.0;
    endState.orientation.w() = 1;

    int maxTime = 5;
    std::vector<trajectory_follower::SubTrajectory> trajectory2D;
    std::vector<trajectory_follower::SubTrajectory> trajectory3D;

    std::cout << "Planning: \n";
    const Planner::PLANNING_RESULT result = planner->plan(base::Time::fromSeconds(maxTime), startState, endState, trajectory2D, trajectory3D);
    std::cout << "Planning Result: " << getResult(result) << std::endl;
    BOOST_CHECK_EQUAL(result, Planner::FOUND_SOLUTION);
}

// DiscreteTheta test
BOOST_AUTO_TEST_CASE(check_discrete_theta_init) {
    DiscreteTheta theta = DiscreteTheta(0, 16);
    BOOST_CHECK_CLOSE(theta.getRadian(), 0, 0.001);

    theta = DiscreteTheta(1, 16);
    BOOST_CHECK_EQUAL(theta.getNumAngles(), 16);
    BOOST_CHECK_CLOSE(theta.getRadian(), 0.3926, 0.001);

    theta = DiscreteTheta(-1, 16);
    BOOST_CHECK_EQUAL(theta.getTheta(), 15);
    BOOST_CHECK_CLOSE(theta.getRadian(), 5.8904, 0.001);

    theta = DiscreteTheta(18, 16);
    BOOST_CHECK_EQUAL(theta.getTheta(), 2);

    theta = DiscreteTheta(16, 16);
    BOOST_CHECK_EQUAL(theta.getTheta(), 0);

    theta = DiscreteTheta(5.90, 16);
    BOOST_CHECK_EQUAL(theta.getTheta(), 15);

    theta = DiscreteTheta(5.45, 16);
    BOOST_CHECK_CLOSE(theta.getRadian(), 5.45, 0.1);
    BOOST_CHECK_EQUAL(theta.getTheta(), 14);

    DiscreteTheta thetaA = DiscreteTheta(3, 16);
    DiscreteTheta thetaB = DiscreteTheta(5, 16);
    BOOST_CHECK_EQUAL(thetaA.shortestDist(thetaB).getTheta(), 2);

    thetaA = DiscreteTheta(2, 16);
    thetaB = DiscreteTheta(14, 16);
    BOOST_CHECK_EQUAL(thetaA.shortestDist(thetaB).getTheta(), 4);
}

BOOST_AUTO_TEST_SUITE_END()