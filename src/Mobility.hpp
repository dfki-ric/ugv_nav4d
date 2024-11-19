#pragma once

namespace ugv_nav4d{

//ATTENTION Mobility is copied from motion_planning_libraries to avoid the dependency. Coping a simple struct is better than getting all the dependencies...

/**
 * Describes the mobility of the system.
 * The minimal turning radius is used to create valid curves.
 * The multipliers are used by SBPL during planning.
 */
struct Mobility {
    // Defines the forward/backward speed of the robot. Is used for cost calculations
    double translationSpeed; // m/sec.
    // Defines the rotational speed of the robot. Is used for cost calculations.
    double rotationSpeed; // rad/sec
    // If > 0 allows to specify the minimal turning radius of the system in meter.
    // Without this not valid curves may be created.
    double minTurningRadius;
    // Resolution used to sample the motion primitive spline
    double spline_sampling_resolution;
    // Remove the goal offset which is there because of the discretization
    bool remove_goal_offset;
    /** The cost of a motion is multiplied by one of the following Multipliers. This allows
      * the user to penalize some motion types.
      * Do ***not*** set the multiplier to zero. If you do all motions of that type will have no cost */
    unsigned int multiplierForward;
    unsigned int multiplierBackward;
    unsigned int multiplierLateral;
    unsigned int multiplierForwardTurn;
    unsigned int multiplierBackwardTurn;
    unsigned int multiplierPointTurn;
    unsigned int multiplierLateralCurve;

    // If > 0 a new goal would be searched, if the choosen goal is invalid
    double searchRadius;
    // The search circle is increased by this value after each unsuccessful case. This option determines the step size between two search radii.
    double searchProgressSteps;

    double maxMotionCurveLength;


    Mobility() :
           translationSpeed(1.0),
           rotationSpeed(1.0),
           minTurningRadius(1.0),
           spline_sampling_resolution(0.05),
           remove_goal_offset(false),
           multiplierForward(1),
           multiplierBackward(2),
           multiplierLateral(4),
           multiplierForwardTurn(2),
           multiplierBackwardTurn(3),
           multiplierPointTurn(3),
           multiplierLateralCurve(4),
           searchRadius(1.0),
           searchProgressSteps(0.1),
           maxMotionCurveLength(100)
    {
    }

    Mobility(double speed,
             double turning_speed,
             double min_turning_radius,
             double sampling_resolution,
             bool correct_goal_offset,
             unsigned int mult_forward=0,
             unsigned int mult_backward=0,
             unsigned int mult_lateral=0,
             unsigned int mult_forward_turn=0,
             unsigned int mult_backward_turn=0,
             unsigned int mult_pointturn=0,
             unsigned int mult_lateral_curve=0,
             double search_radius = 0,
             double search_progress_steps = 0,
             double max_motion_curve_length = 0
            ) :
            translationSpeed(speed),
            rotationSpeed(turning_speed),
            minTurningRadius(min_turning_radius),
            spline_sampling_resolution(sampling_resolution),
            remove_goal_offset(correct_goal_offset),
            multiplierForward(mult_forward),
            multiplierBackward(mult_backward),
            multiplierLateral(mult_lateral),
            multiplierForwardTurn(mult_forward_turn),
            multiplierBackwardTurn(mult_backward_turn),
            multiplierPointTurn(mult_pointturn),
            multiplierLateralCurve(mult_lateral_curve),
            searchRadius(search_radius),
            searchProgressSteps(search_progress_steps),
            maxMotionCurveLength(max_motion_curve_length)
    {
    }
};


}
