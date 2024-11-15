#define BOOST_TEST_MODULE PlannerTestModule
#include <boost/test/included/unit_test.hpp>

#include "ugv_nav4d/Planner.hpp"
#include <base/Angle.hpp>

using namespace ugv_nav4d;

class PlannerTest {
public:
    PlannerTest();
    ~PlannerTest();
    std::string getResult(const Planner::PLANNING_RESULT& result);

    Planner* planner;
    maps::grid::MLSMapSloped* mlsMap;
    PlannerConfig plannerConfig;
    Mobility mobility;
    sbpl_spline_primitives::SplinePrimitivesConfig splinePrimitiveConfig;
    traversability_generator3d::TraversabilityConfig traversabilityConfig;
};

PlannerTest::PlannerTest() {

    splinePrimitiveConfig.gridSize = 0.3;
    splinePrimitiveConfig.numAngles = 16;
    splinePrimitiveConfig.numEndAngles = 8;
    splinePrimitiveConfig.destinationCircleRadius = 6;
    splinePrimitiveConfig.cellSkipFactor = 0.1;
    splinePrimitiveConfig.splineOrder = 4.0;

    mobility.translationSpeed = 0.5;
    mobility.rotationSpeed = 0.5;
    mobility.minTurningRadius = 1;
    mobility.spline_sampling_resolution = 0.05;
    mobility.remove_goal_offset = true;
    mobility.multiplierForward = 1;
    mobility.multiplierBackward = 2;
    mobility.multiplierPointTurn = 3;
    mobility.multiplierLateral = 4;
    mobility.multiplierForwardTurn = 2;
    mobility.multiplierBackwardTurn = 3;
    mobility.multiplierLateralCurve = 4;
    mobility.searchRadius = 1.0;
    mobility.searchProgressSteps = 0.1;
    mobility.maxMotionCurveLength = 100;

    traversabilityConfig.maxStepHeight = 0.2;
    traversabilityConfig.maxSlope = 0.45;
    traversabilityConfig.inclineLimittingMinSlope = 0.2;
    traversabilityConfig.inclineLimittingLimit = 0.1;
    traversabilityConfig.costFunctionDist = 0.0;
    traversabilityConfig.minTraversablePercentage = 0.4;
    traversabilityConfig.robotHeight = 0.5;
    traversabilityConfig.robotSizeX = 0.5;
    traversabilityConfig.robotSizeY = 0.5;
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
    plannerConfig.numThreads = 4;

    maps::grid::Vector2d res(0.3, 0.3);
    maps::grid::Vector2ui numCells(100, 100);

    maps::grid::MLSConfig mls_config;
    mls_config.gapSize = 0.1;
    mls_config.updateModel = maps::grid::MLSConfig::SLOPE;
    
    mlsMap = new maps::grid::MLSMapSloped(numCells, res, mls_config);

    /** Translate the local frame (offset) **/
    mlsMap->getLocalFrame().translation() << 0.5*mlsMap->getSize(), 0;

    Eigen::Vector2d max = 0.5 * mlsMap->getSize();
    Eigen::Vector2d min = -0.5 * mlsMap->getSize();

    for (double x = min.x(); x < max.x(); x += 0.00625)
    {
        for (double y = min.y(); y < max.y(); y += 0.00625)
        {
            mlsMap->mergePoint(Eigen::Vector3d(x, y, 0));
        }
    }
}

PlannerTest::~PlannerTest() {
    delete planner;
    delete mlsMap;
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

BOOST_FIXTURE_TEST_SUITE(PlannerTestSuite, PlannerTest)

BOOST_AUTO_TEST_CASE(check_planner_init_success) {
    planner = new Planner(splinePrimitiveConfig, traversabilityConfig, mobility, plannerConfig);
    BOOST_CHECK(planner != nullptr);
}

BOOST_AUTO_TEST_CASE(check_planner_goal_invalid) {
    planner = new Planner(splinePrimitiveConfig, traversabilityConfig, mobility, plannerConfig);
    BOOST_REQUIRE(planner != nullptr);
    planner->updateMap(*mlsMap);

    base::samples::RigidBodyState startState;
    startState.position.x() = 0;
    startState.position.y() = 0;
    startState.position.z() = 0;
    startState.orientation = Eigen::Quaterniond(1,0,0,0);

    base::samples::RigidBodyState endState;
    endState.position.x() = 40.0;
    endState.position.y() = 0.0;
    endState.position.z() = 0.0;
    endState.orientation = Eigen::Quaterniond(1,0,0,0);

    int maxTime = 5;
    std::vector<trajectory_follower::SubTrajectory> trajectory2D;
    std::vector<trajectory_follower::SubTrajectory> trajectory3D;

    const Planner::PLANNING_RESULT result = planner->plan(base::Time::fromSeconds(maxTime), startState, endState, trajectory2D, trajectory3D);
    BOOST_CHECK_EQUAL(result, Planner::GOAL_INVALID);
}

BOOST_AUTO_TEST_CASE(check_planner_start_invalid) {
    planner = new Planner(splinePrimitiveConfig, traversabilityConfig, mobility, plannerConfig);
    BOOST_REQUIRE(planner != nullptr);
    planner->updateMap(*mlsMap);

    base::samples::RigidBodyState startState;
    startState.position.x() = 40.0;
    startState.position.y() = 0.0;
    startState.position.z() = 0.0;
    startState.orientation = Eigen::Quaterniond(1,0,0,0);

    base::samples::RigidBodyState endState;
    endState.position.x() = 1.0;
    endState.position.y() = 0.0;
    endState.position.z() = 0.0;
    endState.orientation = Eigen::Quaterniond(1,0,0,0);

    int maxTime = 5;
    std::vector<trajectory_follower::SubTrajectory> trajectory2D;
    std::vector<trajectory_follower::SubTrajectory> trajectory3D;

    const Planner::PLANNING_RESULT result = planner->plan(base::Time::fromSeconds(maxTime), startState, endState, trajectory2D, trajectory3D);
    BOOST_CHECK_EQUAL(result, Planner::START_INVALID);
}

BOOST_AUTO_TEST_CASE(check_planner_success) {
    planner = new Planner(splinePrimitiveConfig, traversabilityConfig, mobility, plannerConfig);
    BOOST_REQUIRE(planner != nullptr);
    planner->updateMap(*mlsMap);

    base::samples::RigidBodyState startState;
    startState.position.x() = 2.3;
    startState.position.y() = 4.1;
    startState.position.z() = 0.0;
    startState.orientation = Eigen::Quaterniond(1,0,0,0);

    base::samples::RigidBodyState endState;
    endState.position.x() = 6.1;
    endState.position.y() = 4.2;
    endState.position.z() = 0.0;
    endState.orientation = Eigen::Quaterniond(1,0,0,0);

    int maxTime = 5;
    std::vector<trajectory_follower::SubTrajectory> trajectory2D;
    std::vector<trajectory_follower::SubTrajectory> trajectory3D;

    const Planner::PLANNING_RESULT result = planner->plan(base::Time::fromSeconds(maxTime), startState, endState, trajectory2D, trajectory3D);
    BOOST_CHECK_EQUAL(result, Planner::FOUND_SOLUTION);
    BOOST_CHECK_GT(trajectory2D.size(), 0);
    BOOST_CHECK_GT(trajectory3D.size(), 0);
    BOOST_CHECK_EQUAL(trajectory2D.size(), trajectory3D.size());

    //Start and Goal Positions
    BOOST_REQUIRE(trajectory2D.front().startPose.position.isApprox(startState.position.head<2>(), 1e-6));
    BOOST_REQUIRE(trajectory2D.back().goalPose.position.isApprox(endState.position.head<2>(), 1e-6));
    BOOST_REQUIRE(trajectory2D.front().posSpline.getStartPoint().isApprox(startState.position, 1e-6));
    BOOST_REQUIRE(trajectory2D.back().posSpline.getEndPoint().isApprox(endState.position, 1e-6));

    BOOST_REQUIRE(trajectory3D.front().startPose.position.isApprox(startState.position.head<2>(), 1e-6));
    BOOST_REQUIRE(trajectory3D.back().goalPose.position.isApprox(endState.position.head<2>(), 1e-6));
    BOOST_REQUIRE(trajectory3D.front().posSpline.getStartPoint().isApprox(startState.position, 1e-6));
    BOOST_REQUIRE(trajectory3D.back().posSpline.getEndPoint().isApprox(endState.position, 1e-6));


    //Start and Goal Orientations
    BOOST_REQUIRE_EQUAL(base::Angle::fromRad(trajectory2D.front().startPose.orientation).getRad(),
                        base::Angle::fromRad(base::getYaw(startState.orientation)).getRad());
    BOOST_REQUIRE_EQUAL(base::Angle::fromRad(trajectory2D.back().goalPose.orientation).getRad(),
                        base::Angle::fromRad(base::getYaw(endState.orientation)).getRad());

    BOOST_REQUIRE_EQUAL(base::Angle::fromRad(trajectory3D.front().startPose.orientation).getRad(),
                        base::Angle::fromRad(base::getYaw(startState.orientation)).getRad());
    BOOST_REQUIRE_EQUAL(base::Angle::fromRad(trajectory3D.back().goalPose.orientation).getRad(),
                        base::Angle::fromRad(base::getYaw(endState.orientation)).getRad());

    for (auto& trajectory : trajectory2D){
        BOOST_REQUIRE(trajectory.speed <= mobility.translationSpeed);
        BOOST_REQUIRE(trajectory.speed <= mobility.rotationSpeed);

        if (trajectory.driveMode != trajectory_follower::ModeTurnOnTheSpot){
            BOOST_REQUIRE(trajectory.posSpline.isEmpty() == false);

            BOOST_REQUIRE(trajectory.getCurvatureMax() <= mobility.minTurningRadius/2);
            BOOST_REQUIRE(trajectory.posSpline.getCurveLength() <= mobility.maxMotionCurveLength);               
        }
        else{
            BOOST_REQUIRE(trajectory.orientationSpline.isEmpty() == false);

            BOOST_CHECK_CLOSE_FRACTION(base::Angle::fromRad(trajectory.getIntermediatePoint(trajectory.orientationSpline.getStartParam()).orientation).getRad(),
                                base::Angle::fromRad(trajectory.startPose.orientation).getRad(), 1e-6);

            BOOST_CHECK_CLOSE_FRACTION(base::Angle::fromRad(trajectory.getIntermediatePoint(trajectory.orientationSpline.getEndParam()).orientation).getRad(),
                                base::Angle::fromRad(trajectory.goalPose.orientation).getRad(), 1e-6);
        }
    }

    for (auto& trajectory : trajectory3D){
        BOOST_REQUIRE(trajectory.speed <= mobility.translationSpeed);
        BOOST_REQUIRE(trajectory.speed <= mobility.rotationSpeed);

        if (trajectory.driveMode != trajectory_follower::ModeTurnOnTheSpot){
            BOOST_REQUIRE(trajectory.posSpline.isEmpty() == false);

            BOOST_REQUIRE(trajectory.getCurvatureMax() <= mobility.minTurningRadius/2);
            BOOST_REQUIRE(trajectory.posSpline.getCurveLength() <= mobility.maxMotionCurveLength);               
        }
        else{
            BOOST_REQUIRE(trajectory.orientationSpline.isEmpty() == false);

            BOOST_CHECK_CLOSE_FRACTION(base::Angle::fromRad(trajectory.getIntermediatePoint(trajectory.orientationSpline.getStartParam()).orientation).getRad(),
                                base::Angle::fromRad(trajectory.startPose.orientation).getRad(), 1e-6);

            BOOST_CHECK_CLOSE_FRACTION(base::Angle::fromRad(trajectory.getIntermediatePoint(trajectory.orientationSpline.getEndParam()).orientation).getRad(),
                                base::Angle::fromRad(trajectory.goalPose.orientation).getRad(), 1e-6);
        }
    }


}


BOOST_AUTO_TEST_SUITE_END()