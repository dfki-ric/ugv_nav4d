#define BOOST_TEST_MODULE EnvironmentXYZThetaTestModule
#include <boost/test/included/unit_test.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include "ugv_nav4d/EnvironmentXYZTheta.hpp"

using namespace ugv_nav4d;
using namespace maps::grid;

std::vector<Eigen::Vector3d> startPositions;
std::vector<Eigen::Vector3d> goalPositions;

struct EnvironmentXYZThetaTest {
    EnvironmentXYZThetaTest(){}
    ~EnvironmentXYZThetaTest(){}


    EnvironmentXYZTheta* environment;
    Mobility mobility;
    sbpl_spline_primitives::SplinePrimitivesConfig splinePrimitiveConfig;
    traversability_generator3d::TraversabilityConfig traversabilityConfig;
    bool map_loaded = false;
};

BOOST_FIXTURE_TEST_CASE(travmap_resolution_not_equal_to_mls_resolution, EnvironmentXYZThetaTest) {
    Vector2d res(0.3, 0.3);
    Vector2ui numCells(10, 10);

    MLSConfig mls_config;
    mls_config.gapSize = 0.1;
    mls_config.updateModel = MLSConfig::SLOPE;
    MLSMapSloped mls_o = MLSMapSloped(numCells, res, mls_config);

    /** Translate the local frame (offset) **/
    mls_o.getLocalFrame().translation() << 0.5*mls_o.getSize(), 0;

    Eigen::Vector2d max = 0.5 * mls_o.getSize();
    Eigen::Vector2d min = -0.5 * mls_o.getSize();
    for (double x = min.x(); x < max.x(); x += 0.00625)
    {
        //double cs = std::cos(x * M_PI/2.5);
        for (double y = min.y(); y < max.y(); y += 0.00625)
        {
            //double sn = std::sin(y * M_PI/2.5);
            mls_o.mergePoint(Eigen::Vector3d(x, y, 0));
        }
    }

    std::shared_ptr<maps::grid::MLSMapSloped> mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mls_o);
    traversabilityConfig.gridResolution = 0.5;
    splinePrimitiveConfig.gridSize = 0.5;
    environment = new EnvironmentXYZTheta(mlsPtr, traversabilityConfig, splinePrimitiveConfig, mobility);
    std::vector<Eigen::Vector3d> startPositions;
    startPositions.emplace_back(Eigen::Vector3d(0,0,0));
    environment->expandMap(startPositions);
    BOOST_CHECK_EQUAL(environment->getTravGen().getNumNodes(), 1);
    delete environment;

    traversabilityConfig.gridResolution = 0.1;
    splinePrimitiveConfig.gridSize = 0.1;
    environment = new EnvironmentXYZTheta(mlsPtr, traversabilityConfig, splinePrimitiveConfig, mobility);
    environment->expandMap(startPositions);
    BOOST_CHECK_EQUAL(environment->getTravGen().getNumNodes(), 9);
}

BOOST_FIXTURE_TEST_CASE(travmap_resolution_equal_to_mls_resolution, EnvironmentXYZThetaTest) {
    Vector2d res(0.3, 0.3);
    Vector2ui numCells(10, 10);

    MLSConfig mls_config;
    mls_config.gapSize = 0.1;
    mls_config.updateModel = MLSConfig::SLOPE;
    MLSMapSloped mls_o = MLSMapSloped(numCells, res, mls_config);

    /** Translate the local frame (offset) **/
    mls_o.getLocalFrame().translation() << 0.5*mls_o.getSize(), 0;

    Eigen::Vector2d max = 0.5 * mls_o.getSize();
    Eigen::Vector2d min = -0.5 * mls_o.getSize();

    for (double x = min.x(); x < max.x(); x += 0.00625)
    {
        //double cs = std::cos(x * M_PI/2.5);
        for (double y = min.y(); y < max.y(); y += 0.00625)
        {
            //double sn = std::sin(y * M_PI/2.5);
            mls_o.mergePoint(Eigen::Vector3d(x, y, 0));
        }
    }

    std::shared_ptr<maps::grid::MLSMapSloped> mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mls_o);
    traversabilityConfig.gridResolution = 0.3;
    splinePrimitiveConfig.gridSize = 0.3;

    environment = new EnvironmentXYZTheta(mlsPtr, traversabilityConfig, splinePrimitiveConfig, mobility);
    std::vector<Eigen::Vector3d> startPositions;
    startPositions.emplace_back(Eigen::Vector3d(0,0,0));
    environment->expandMap(startPositions);
    BOOST_CHECK_EQUAL(environment->getTravGen().getNumNodes(), numCells.x()* numCells.y());
}

BOOST_FIXTURE_TEST_CASE(check_travmap, EnvironmentXYZThetaTest) {
    Vector2d res(0.3, 0.3);
    Vector2ui numCells(10, 10);

    MLSConfig mls_config;
    mls_config.gapSize = 0.1;
    mls_config.updateModel = MLSConfig::SLOPE;
    MLSMapSloped mls_o = MLSMapSloped(numCells, res, mls_config);

    /** Translate the local frame (offset) **/
    mls_o.getLocalFrame().translation() << 0.5*mls_o.getSize(), 0;

    Eigen::Vector2d max = 0.5 * mls_o.getSize();
    Eigen::Vector2d min = -0.5 * mls_o.getSize();

    for (double x = min.x(); x < max.x(); x += 0.00625)
    {
        for (double y = min.y(); y < max.y(); y += 0.00625)
        {
            double z = 0;
            if ((x >= 0.6 && x < 0.9) && (y >= 0.6 && y < 0.9)){
                z = 0.3;
            }
            mls_o.mergePoint(Eigen::Vector3d(x, y, z));
        }
    }

    std::shared_ptr<maps::grid::MLSMapSloped> mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mls_o);
    traversabilityConfig.gridResolution = 0.3;
    splinePrimitiveConfig.gridSize = 0.3;

    environment = new EnvironmentXYZTheta(mlsPtr, traversabilityConfig, splinePrimitiveConfig, mobility);
    std::vector<Eigen::Vector3d> startPositions;
    startPositions.emplace_back(Eigen::Vector3d(0,0,0));
    environment->expandMap(startPositions);

    Eigen::Vector3d positionFron{0.9, 0.3, 0};
    maps::grid::Index idxFrontierNode;
    environment->getTravGen().getTraversabilityMap()->toGrid(positionFron, idxFrontierNode);
    auto frontier = environment->getTravGen().findMatchingTraversabilityPatchAt(idxFrontierNode,0);
    BOOST_CHECK_EQUAL(frontier->getType(), ::maps::grid::TraversabilityNodeBase::FRONTIER);

    Eigen::Vector3d positionTrav{0.3, 0.3, 0};
    maps::grid::Index idxTraversableNode;
    environment->getTravGen().getTraversabilityMap()->toGrid(positionTrav, idxTraversableNode);
    auto traversable = environment->getTravGen().findMatchingTraversabilityPatchAt(idxTraversableNode,0);
    BOOST_CHECK_EQUAL(traversable->getType(), ::maps::grid::TraversabilityNodeBase::TRAVERSABLE);

    Eigen::Vector3d positionObs{0.65, 0.65, 0};
    maps::grid::Index idxObstacleNode;
    environment->getTravGen().getTraversabilityMap()->toGrid(positionObs, idxObstacleNode);
    auto &trList(environment->getTravGen().getTraversabilityMap()->at(idxObstacleNode));
    for(auto *snode : trList)
    {
        BOOST_CHECK_EQUAL(snode->getType(), ::maps::grid::TraversabilityNodeBase::OBSTACLE);
    }
}

BOOST_FIXTURE_TEST_CASE(check_stepheight, EnvironmentXYZThetaTest) {
    Vector2d res(0.3, 0.3);
    Vector2ui numCells(10, 10);

    MLSConfig mls_config;
    mls_config.gapSize = 0.1;
    mls_config.updateModel = MLSConfig::SLOPE;
    MLSMapSloped mls_o = MLSMapSloped(numCells, res, mls_config);

    /** Translate the local frame (offset) **/
    mls_o.getLocalFrame().translation() << 0.5*mls_o.getSize(), 0;

    Eigen::Vector2d max = 0.5 * mls_o.getSize();
    Eigen::Vector2d min = -0.5 * mls_o.getSize();

    for (double x = min.x(); x < max.x(); x += 0.00625)
    {
        for (double y = min.y(); y < max.y(); y += 0.00625)
        {
            double z = 0;
            if ((x >= 0 && x < 0.3) && (y >= 0 && y < 0.3)){
                z = 0.1;
            }
            mls_o.mergePoint(Eigen::Vector3d(x, y, z));
        }
    }

    std::shared_ptr<maps::grid::MLSMapSloped> mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mls_o);
    traversabilityConfig.gridResolution = 0.3;
    splinePrimitiveConfig.gridSize = 0.3;

    environment = new EnvironmentXYZTheta(mlsPtr, traversabilityConfig, splinePrimitiveConfig, mobility);
    std::vector<Eigen::Vector3d> startPositions;
    startPositions.emplace_back(Eigen::Vector3d(0,0,0));
    environment->expandMap(startPositions);

    Eigen::Vector3d positionObs{0.25, 0.25, 0};
    maps::grid::Index idxObstacleNode;

    environment->getTravGen().getTraversabilityMap()->toGrid(positionObs, idxObstacleNode);
    for(auto *snode : environment->getTravGen().getTraversabilityMap()->at(idxObstacleNode))
    {
        BOOST_CHECK_EQUAL(snode->getType(), ::maps::grid::TraversabilityNodeBase::OBSTACLE);
    }

    delete environment;

    traversabilityConfig.maxStepHeight = 0.2;
    environment = new EnvironmentXYZTheta(mlsPtr, traversabilityConfig, splinePrimitiveConfig, mobility);
    environment->expandMap(startPositions);

    environment->getTravGen().getTraversabilityMap()->toGrid(positionObs, idxObstacleNode);
    for(auto *snode : environment->getTravGen().getTraversabilityMap()->at(idxObstacleNode))
    {
        BOOST_CHECK_EQUAL(snode->getType(), ::maps::grid::TraversabilityNodeBase::TRAVERSABLE);
    }
}

BOOST_FIXTURE_TEST_CASE(set_start_and_goal, EnvironmentXYZThetaTest) {
    Vector2d res(0.3, 0.3);
    Vector2ui numCells(100, 100);

    MLSConfig mls_config;
    mls_config.gapSize = 0.1;
    mls_config.updateModel = MLSConfig::SLOPE;
    MLSMapSloped mls_o = MLSMapSloped(numCells, res, mls_config);

    /** Translate the local frame (offset) **/
    mls_o.getLocalFrame().translation() << 0.5*mls_o.getSize(), 0;

    Eigen::Vector2d max = 0.5 * mls_o.getSize();
    Eigen::Vector2d min = -0.5 * mls_o.getSize();

    for (double x = min.x(); x < max.x(); x += 0.00625)
    {
        for (double y = min.y(); y < max.y(); y += 0.00625)
        {
            mls_o.mergePoint(Eigen::Vector3d(x, y, 0));
        }
    }

    std::shared_ptr<maps::grid::MLSMapSloped> mlsPtr = std::make_shared<maps::grid::MLSMapSloped>(mls_o);
    traversabilityConfig.gridResolution = 0.3;
    splinePrimitiveConfig.gridSize = 0.3;

    environment = new EnvironmentXYZTheta(mlsPtr, traversabilityConfig, splinePrimitiveConfig, mobility);
    std::vector<Eigen::Vector3d> startPositions;
    startPositions.emplace_back(Eigen::Vector3d(0,0,0));
    environment->expandMap(startPositions);

    Eigen::Vector3d startPos{0,0,0};
    Eigen::Vector3d goalPos{12, 12, 0};

    environment->setStart(startPos,0);
    environment->setGoal(goalPos,0);
}
