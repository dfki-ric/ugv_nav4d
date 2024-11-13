#define BOOST_TEST_MODULE DiscreteThetaModule

#include "ugv_nav4d/DiscreteTheta.hpp"

#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_CASE(check_discrete_theta_init) {
    DiscreteTheta theta = DiscreteTheta(0, 16);
    BOOST_CHECK_CLOSE_FRACTION(theta.getRadian(), 0, 0.01);

    theta = DiscreteTheta(1, 16);
    BOOST_CHECK_EQUAL(theta.getNumAngles(), 16);
    BOOST_CHECK_CLOSE_FRACTION(theta.getRadian(), 0.3926, 0.01);

    theta = DiscreteTheta(-1, 16);
    BOOST_CHECK_EQUAL(theta.getTheta(), 15);
    BOOST_CHECK_CLOSE_FRACTION(theta.getRadian(), 5.8904, 0.01);

    theta = DiscreteTheta(18, 16);
    BOOST_CHECK_EQUAL(theta.getTheta(), 2);

    theta = DiscreteTheta(16, 16);
    BOOST_CHECK_EQUAL(theta.getTheta(), 0);

    theta = DiscreteTheta(5.90, 16);
    BOOST_CHECK_EQUAL(theta.getTheta(), 15);

    theta = DiscreteTheta(5.45, 16);
    BOOST_CHECK_CLOSE_FRACTION(theta.getRadian(), 5.45, 0.01);
    BOOST_CHECK_EQUAL(theta.getTheta(), 14);

    DiscreteTheta thetaA = DiscreteTheta(3, 16);
    DiscreteTheta thetaB = DiscreteTheta(5, 16);
    BOOST_CHECK_EQUAL(thetaA.shortestDist(thetaB).getTheta(), 2);

    thetaA = DiscreteTheta(2, 16);
    thetaB = DiscreteTheta(14, 16);
    BOOST_CHECK_EQUAL(thetaA.shortestDist(thetaB).getTheta(), 4);
}