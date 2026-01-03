#define BOOST_TEST_MODULE EnvironmentXYZThetaTestModule
#include <boost/test/included/unit_test.hpp>

#include "ugv_nav4d/EnvironmentXYZTheta.hpp"
#include "traversability_generator3d/TraversabilityGenerator3d.hpp"

#include <memory>

using namespace ugv_nav4d;
using namespace maps::grid;

struct EnvironmentXYZThetaTest
{
    traversability_generator3d::TraversabilityConfig travConf;
    sbpl_spline_primitives::SplinePrimitivesConfig primitiveConf;
    Mobility mobility;

    std::unique_ptr<traversability_generator3d::TraversabilityGenerator3d> travGen;
    std::shared_ptr<traversability_generator3d::TravMap3d> travMap;
    std::unique_ptr<EnvironmentXYZTheta> env;

    EnvironmentXYZThetaTest()
    {
        travConf.gridResolution = 0.3;
        primitiveConf.gridSize  = 0.3;
        buildFlatMap();
    }

    void buildTravMapFromMLS(const MLSMapSloped& mls)
    {
        travConf.gridResolution = 0.3;
        travConf.robotSizeX = 0.6;
        travConf.robotSizeY = 0.6;
        travConf.robotHeight = 0.5;

        travGen.reset(new traversability_generator3d::TraversabilityGenerator3d(travConf));

        std::shared_ptr<MLSMapSloped> mlsPtr =
            std::make_shared<MLSMapSloped>(mls);

        travGen->setMLSGrid(mlsPtr);

        const Eigen::Vector3d start{0.0, 0.0, 0.0};
        travGen->expandAll(start);

        travMap = std::make_shared<traversability_generator3d::TravMap3d>(
            travGen->getTraversabilityMap()
        );
    }

    void buildFlatMap()
    {
        Vector2d res(0.3,0.3);
        Vector2ui cells(60,60);

        MLSConfig cfg;
        cfg.updateModel = MLSConfig::SLOPE;

        MLSMapSloped mls(cells, res, cfg);
        mls.getLocalFrame().translation() << 0.5 * mls.getSize(), 0;

        for (double x = -9; x <= 9; x += 0.1)
            for (double y = -9; y <= 9; y += 0.1)
                mls.mergePoint({x, y, 0.0});

        buildTravMapFromMLS(mls);
    }

    void buildInclinedMap(double slope)
    {
        Vector2d res(0.3,0.3);
        Vector2ui cells(60,60);

        MLSConfig cfg;
        cfg.updateModel = MLSConfig::SLOPE;

        MLSMapSloped mls(cells,res,cfg);
        mls.getLocalFrame().translation() << 0.5 * mls.getSize(), 0;

        for(double x=-8;x<=8;x+=0.1)
            for(double y=-8;y<=8;y+=0.1)
                mls.mergePoint({x,y,slope*x});

        buildTravMapFromMLS(mls);
    }
};

BOOST_FIXTURE_TEST_CASE(set_start_and_goal, EnvironmentXYZThetaTest)
{
    env.reset(new EnvironmentXYZTheta(
        travMap, travConf, primitiveConf, mobility));

    BOOST_REQUIRE_NO_THROW(env->setStart({0,0,0}, 0.0));
    BOOST_REQUIRE_NO_THROW(env->setGoal({3,3,0}, 0.0));
}

BOOST_FIXTURE_TEST_CASE(goal_outside_map_throws, EnvironmentXYZThetaTest)
{
    EnvironmentXYZTheta env(travMap, travConf, primitiveConf, mobility);
    env.setStart({0,0,0}, 0.0);

    BOOST_CHECK_THROW(
        env.setGoal({1000,1000,0}, 0.0),
        std::runtime_error
    );
}

BOOST_FIXTURE_TEST_CASE(goal_heuristic_is_finite, EnvironmentXYZThetaTest)
{
    EnvironmentXYZTheta env(travMap, travConf, primitiveConf, mobility);
    env.setStart({0,0,0}, 0.0);
    env.setGoal({3,3,0}, 0.0);

    int h = env.GetGoalHeuristic(0);
    BOOST_CHECK(h >= 0);
    BOOST_CHECK(h < std::numeric_limits<int>::max());
}

BOOST_FIXTURE_TEST_CASE(successors_are_generated, EnvironmentXYZThetaTest)
{
    EnvironmentXYZTheta env(travMap, travConf, primitiveConf, mobility);
    env.setStart({0,0,0}, 0.0);
    env.setGoal({3,3,0}, 0.0);

    std::vector<int> succ;
    std::vector<int> cost;

    env.GetSuccs(0, &succ, &cost);

    BOOST_CHECK(!succ.empty());
    BOOST_CHECK_EQUAL(succ.size(), cost.size());
}

BOOST_FIXTURE_TEST_CASE(motion_extraction_valid, EnvironmentXYZThetaTest)
{
    EnvironmentXYZTheta env(travMap, travConf, primitiveConf, mobility);
    env.setStart({0,0,0}, 0.0);
    env.setGoal({2,0,0}, 0.0);

    std::vector<int> succ;
    std::vector<int> cost;
    std::vector<size_t> motionIds;

    env.GetSuccs(0, &succ, &cost, motionIds);

    BOOST_REQUIRE(!succ.empty());

    const Motion& m = env.getMotion(0, succ.front());
    BOOST_CHECK(m.baseCost > 0);
}

BOOST_FIXTURE_TEST_CASE(state_space_grows, EnvironmentXYZThetaTest)
{
    EnvironmentXYZTheta env(travMap, travConf, primitiveConf, mobility);
    env.setStart({0,0,0}, 0.0);
    env.setGoal({5,5,0}, 0.0);

    std::vector<int> succ;
    std::vector<int> cost;

    env.GetSuccs(0, &succ, &cost);

    BOOST_CHECK(env.SizeofCreatedEnv() > 1);
}

BOOST_FIXTURE_TEST_CASE(goal_before_start_throws, EnvironmentXYZThetaTest)
{
    EnvironmentXYZTheta env(travMap, travConf, primitiveConf, mobility);

    BOOST_CHECK_THROW(
        env.setGoal({1,1,0}, 0.0),
        std::runtime_error
    );
}

BOOST_FIXTURE_TEST_CASE(theta_discretization, EnvironmentXYZThetaTest)
{
    EnvironmentXYZTheta env(travMap, travConf, primitiveConf, mobility);

    env.setStart({0,0,0}, M_PI/4);
    env.setGoal({2,0,0}, M_PI/2);

    std::vector<int> succ;
    std::vector<int> cost;
    std::vector<size_t> motionIds;

    env.GetSuccs(0, &succ, &cost, motionIds);

    BOOST_CHECK(!motionIds.empty());
}

BOOST_FIXTURE_TEST_CASE(motion_cost_positive, EnvironmentXYZThetaTest)
{
    EnvironmentXYZTheta env(travMap, travConf, primitiveConf, mobility);
    env.setStart({0,0,0}, 0);
    env.setGoal({3,0,0}, 0);

    std::vector<int> succ;
    std::vector<int> cost;

    env.GetSuccs(0, &succ, &cost);

    for (int c : cost)
    {
        BOOST_CHECK(c > 0);
    }
}

BOOST_FIXTURE_TEST_CASE(state_position_matches_map, EnvironmentXYZThetaTest)
{
    EnvironmentXYZTheta env(travMap, travConf, primitiveConf, mobility);
    env.setStart({1,2,0}, 0);
    env.setGoal({3,4,0}, 0);

    maps::grid::Vector3d pos = env.getStatePosition(0);

    BOOST_CHECK_CLOSE(pos.x(), 1.0, 10);
    BOOST_CHECK_CLOSE(pos.y(), 2.0, 10);
}

BOOST_FIXTURE_TEST_CASE(trajectory_generation, EnvironmentXYZThetaTest)
{
    EnvironmentXYZTheta env(travMap, travConf, primitiveConf, mobility);
    env.setStart({0,0,0}, 0);
    env.setGoal({3,0,0}, 0);

    std::vector<int> succ;
    std::vector<int> cost;
    env.GetSuccs(0, &succ, &cost);

    std::vector<int> path{0, succ.front()};
    std::vector<trajectory_follower::SubTrajectory> traj;

    env.getTrajectory(
        path, traj,
        true,
        {0,0,0},
        {3,0,0},
        0.0
    );

    BOOST_CHECK(!traj.empty());
}

BOOST_FIXTURE_TEST_CASE(successors_generated, EnvironmentXYZThetaTest)
{
    EnvironmentXYZTheta env(travMap, travConf,
                            primitiveConf, mobility);

    env.setStart({0,0,0}, 0);
    env.setGoal({3,0,0}, 0);

    std::vector<int> succ, cost;
    env.GetSuccs(0, &succ, &cost);

    BOOST_CHECK(!succ.empty());
    BOOST_CHECK_EQUAL(succ.size(), cost.size());
}

BOOST_FIXTURE_TEST_CASE(successor_costs_positive, EnvironmentXYZThetaTest)
{
    EnvironmentXYZTheta env(travMap, travConf,
                            primitiveConf, mobility);

    env.setStart({0,0,0}, 0);
    env.setGoal({3,0,0}, 0);

    std::vector<int> succ, cost;
    env.GetSuccs(0, &succ, &cost);

    for (int c : cost)
        BOOST_CHECK(c > 0);
}