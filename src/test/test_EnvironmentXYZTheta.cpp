#define BOOST_TEST_MODULE EnvironmentXYZThetaTestModule

#include <fstream>
#include <cstdlib>

#include "ugv_nav4d/EnvironmentXYZTheta.hpp"
#include "ugv_nav4d/Mobility.hpp"
#include <sbpl_spline_primitives/SplinePrimitivesConfig.hpp>
#include <traversability_generator3d/TraversabilityConfig.hpp>

#include <maps/grid/MLSMap.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <boost/test/included/unit_test.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

using namespace ugv_nav4d;

std::string mlsBinFile;

// Define EnvironmentXYZThetaTest fixture for Boost Test
struct EnvironmentXYZThetaTest {
    EnvironmentXYZThetaTest();
    ~EnvironmentXYZThetaTest();

    void loadMlsMap();

    typedef EnvironmentXYZTheta::MLGrid MLSBase;

    EnvironmentXYZTheta* environment;
    maps::grid::MLSMapSloped mlsMap;
    Mobility mobility;
    sbpl_spline_primitives::SplinePrimitivesConfig splinePrimitiveConfig;
    traversability_generator3d::TraversabilityConfig traversabilityConfig;
    bool map_loaded = false;
};

EnvironmentXYZThetaTest::EnvironmentXYZThetaTest() {
    std::cout << "Called SetUp()" << std::endl;

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
    mobility.multiplierBackward = 3;
    mobility.multiplierPointTurn = 3;
    mobility.multiplierLateral = 4;
    mobility.multiplierForwardTurn = 2;
    mobility.multiplierBackwardTurn = 4;
    mobility.multiplierLateralCurve = 4;
    mobility.searchRadius = 1.0;
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

    loadMlsMap();
}

EnvironmentXYZThetaTest::~EnvironmentXYZThetaTest(){
    std::cout << "Called TearDown()" << std::endl;
    delete environment;  
}

void EnvironmentXYZThetaTest::loadMlsMap() {
    std::string filename;

    if (mlsBinFile.empty()) {
        filename = "cave_circuit_mls.bin";
    } else {
        filename = mlsBinFile;
    }

    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        map_loaded = false;
        return;
    }

    try {
        boost::archive::binary_iarchive ia(file);
        ia >> mlsMap;  // Deserialize into mlsMap
    } catch (const std::exception& e) {
        map_loaded = false;
    }
    map_loaded = true;
}

// Define the test suite and test cases
BOOST_FIXTURE_TEST_SUITE(EnvironmentXYZThetaTestSuite, EnvironmentXYZThetaTest)

BOOST_AUTO_TEST_CASE(check_planner_init_failure_wrong_grid_resolutions) {
    std::shared_ptr<MLSBase> mlsPtr = std::make_shared<MLSBase>(mlsMap);
    try {
        environment = new EnvironmentXYZTheta(mlsPtr, traversabilityConfig, splinePrimitiveConfig, mobility);
    } catch (const std::exception& ex) {
        std::cout << "Exception: \n" << ex.what() << "\n";
        BOOST_CHECK_NE(splinePrimitiveConfig.gridSize, traversabilityConfig.gridResolution);
    }
}

BOOST_AUTO_TEST_SUITE_END()