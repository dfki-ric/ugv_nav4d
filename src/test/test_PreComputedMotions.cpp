#define BOOST_TEST_MODULE PreComputedMotionsModule

#include "ugv_nav4d/PreComputedMotions.hpp"
#include "ugv_nav4d/Mobility.hpp"
#include <sbpl_spline_primitives/SplinePrimitivesConfig.hpp>

#include <boost/test/included/unit_test.hpp>

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
    motions->computeMotions(splinePrimitiveConfig.gridSize, 0.3);

    delete motions;
}