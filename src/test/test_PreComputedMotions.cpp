#define BOOST_TEST_MODULE PreComputedMotionsModule
#include <boost/test/included/unit_test.hpp>

#include "ugv_nav4d/PreComputedMotions.hpp"
#include <base/Pose.hpp>

using namespace ugv_nav4d;

BOOST_AUTO_TEST_CASE(initialization) {

    sbpl_spline_primitives::SplinePrimitivesConfig splinePrimitiveConfig;
    Mobility mobilityConfig;

    PreComputedMotions* motions = new PreComputedMotions(splinePrimitiveConfig,mobilityConfig);
    delete motions;
}

BOOST_AUTO_TEST_CASE(calculate_curvature_from_radius) {

    sbpl_spline_primitives::SplinePrimitivesConfig splinePrimitiveConfig;
    Mobility mobilityConfig;

    PreComputedMotions* motions = new PreComputedMotions(splinePrimitiveConfig,mobilityConfig);
    double curvature = motions->calculateCurvatureFromRadius(2.0);
    BOOST_CHECK_CLOSE_FRACTION(curvature, 0.5, 0.001);

    delete motions;
}

BOOST_AUTO_TEST_CASE(generate_motions) {

    sbpl_spline_primitives::SplinePrimitivesConfig splinePrimitiveConfig;
    Mobility mobilityConfig;
    mobilityConfig.minTurningRadius = 1.0;
    
    PreComputedMotions* motions = new PreComputedMotions(splinePrimitiveConfig,mobilityConfig);
    motions->computeMotions(splinePrimitiveConfig.gridSize, splinePrimitiveConfig.gridSize);
    delete motions;
}

BOOST_AUTO_TEST_CASE(check_motions) {

    sbpl_spline_primitives::SplinePrimitivesConfig splinePrimitiveConfig;
    Mobility mobilityConfig;
    mobilityConfig.minTurningRadius = 1.0;
    
    PreComputedMotions* motions = new PreComputedMotions(splinePrimitiveConfig,mobilityConfig);
    motions->computeMotions(splinePrimitiveConfig.gridSize, splinePrimitiveConfig.gridSize);

    //We generate primitives for 16 start angles.
    for (int i = 1; i <= 16; ++i) {
        DiscreteTheta theta(i,16);
        std::vector<Motion> motions_start = motions->getMotionForStartTheta(theta);
        BOOST_REQUIRE(motions_start.size() > 0);
        for (const auto& motion : motions_start) {
            BOOST_REQUIRE(motion.baseCost > 0);
            BOOST_REQUIRE(motion.costMultiplier > 0);
            BOOST_REQUIRE(motion.costScaleFactor > 0);

            switch (motion.type){
                case Motion::MOV_FORWARD:
                    BOOST_CHECK_GT(motion.fullSplineSamples.size(), 0);
                    BOOST_CHECK_GT(motion.intermediateStepsTravMap.size(), 0);
                    BOOST_CHECK_GT(motion.intermediateStepsObstMap.size(), 0);
                    break;
                case Motion::MOV_BACKWARD:
                    BOOST_CHECK_GT(motion.fullSplineSamples.size(), 0);
                    BOOST_CHECK_GT(motion.intermediateStepsTravMap.size(), 0);
                    BOOST_CHECK_GT(motion.intermediateStepsObstMap.size(), 0);
                    break;
                case Motion::MOV_LATERAL:
                    BOOST_CHECK_GT(motion.fullSplineSamples.size(), 0);
                    BOOST_CHECK_GT(motion.intermediateStepsTravMap.size(), 0);
                    BOOST_CHECK_GT(motion.intermediateStepsObstMap.size(), 0);
                    BOOST_CHECK_EQUAL(motion.startTheta.getRadian(), 
                                      motion.endTheta.getRadian());
                    break;
                case Motion:: MOV_POINTTURN:
                    BOOST_CHECK_EQUAL(motion.fullSplineSamples.size(), 0);
                    BOOST_CHECK_NE(motion.startTheta.getRadian(), motion.endTheta.getRadian());
                    break;
                default:
                    throw std::runtime_error("test_PreComputedMotions: Invalid motion type detectd!");
                    break;

            }

            //Point-turns do not have any splines
            if (motion.type != Motion::MOV_POINTTURN){
                for (const auto& cellWithPoses : motion.fullSplineSamples){
                    BOOST_CHECK_GT(cellWithPoses.poses.size(), 0);
                }

                base::Angle splineStartAngle = base::Angle::fromRad(motion.fullSplineSamples.front().poses.front().orientation);
                base::Angle splineEndAngle = base::Angle::fromRad(motion.fullSplineSamples.back().poses.back().orientation);

                base::Angle motionStartAngle = base::Angle::fromRad(motion.startTheta.getRadian());
                base::Angle motionEndAngle = base::Angle::fromRad(motion.endTheta.getRadian());

                if (motion.type == Motion::MOV_FORWARD){
                    BOOST_CHECK_CLOSE_FRACTION(splineStartAngle.getRad(), motionStartAngle.getRad(), 0.01);   
                    BOOST_CHECK_CLOSE_FRACTION(splineEndAngle.getRad(), motionEndAngle.getRad(), 0.01); 
                }

                //Backward motions have a 180 degree angle diff between spline and motion
                else if (motion.type == Motion::MOV_BACKWARD){
                    BOOST_CHECK_CLOSE_FRACTION(std::abs((splineStartAngle - motionStartAngle).getRad()), 3.142, 0.01);   
                    BOOST_CHECK_CLOSE_FRACTION(std::abs((splineEndAngle - motionEndAngle).getRad()), 3.142, 0.01);  
                }

                //Lateral motions have a 90 degree angle diff between spline and motion (robot maintains heading)
                else if (motion.type == Motion::MOV_LATERAL){
                    BOOST_CHECK_CLOSE_FRACTION(std::abs((splineStartAngle - motionStartAngle).getRad()), 1.57, 0.01);   
                    BOOST_CHECK_CLOSE_FRACTION(std::abs((splineEndAngle - motionEndAngle).getRad()), 1.57, 0.01);             
                }

                const base::Pose2D& splineFinalPosition = motion.fullSplineSamples.back().poses.back();
                const base::Pose2D& travMapFinalPosition = motion.intermediateStepsTravMap.back().pose;
                const base::Pose2D& obstMapFinalPosition = motion.intermediateStepsObstMap.back().pose;

                //Check if end position of full spline and sampled spline are same
                BOOST_REQUIRE(splineFinalPosition.position.isApprox(travMapFinalPosition.position, 1e-6));
                BOOST_REQUIRE(splineFinalPosition.position.isApprox(obstMapFinalPosition.position, 1e-6));
            }
        }    
    }

    delete motions;
}

BOOST_AUTO_TEST_CASE(calculate_cost) {

    sbpl_spline_primitives::SplinePrimitivesConfig splinePrimitiveConfig;
    Mobility mobilityConfig;

    PreComputedMotions* motions = new PreComputedMotions(splinePrimitiveConfig,mobilityConfig);
    motions->computeMotions(splinePrimitiveConfig.gridSize, splinePrimitiveConfig.gridSize);

    //Start angle 0 degrees (First set of primitives from 16 sets)
    DiscreteTheta theta(1,16);
    std::vector<Motion> motions_start = motions->getMotionForStartTheta(theta);
    BOOST_REQUIRE(motions_start.size() > 0);
    BOOST_CHECK_THROW(motions_start[0].calculateCost(0,0,0,0,0), std::runtime_error);

    //Translation
    BOOST_CHECK_GT(motions_start[0].calculateCost(1,0,0.1,0.1,1), 0);
    //Rotation
    BOOST_CHECK_GT(motions_start[0].calculateCost(0,1,0.1,0.1,1), 0);
    //Translation and rotation
    BOOST_CHECK_GT(motions_start[0].calculateCost(1,1,0.1,0.1,1), 0);
   
    delete motions;
}