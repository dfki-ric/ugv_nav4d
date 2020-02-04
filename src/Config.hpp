#pragma once

namespace ugv_nav4d{

/** Cost function parameters for frontier exploration */
struct FrontierGeneratorParameters
{
    /** How important is the distance from the node to the goal position  */
    double distToGoalFactor = 1.0;
    /** How important is the distance from the start node */
    double distFromStartFactor = 1.0;
    /**How important is the size of the explorable area around the frontier node */
    double explorableFactor = 1.0;
    /**The radius of patches that will be visited when calculating the number of explorable patches in the vicinity of a patch */
    unsigned visitRadius = 3;
};


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
    
    
    Mobility() : 
           translationSpeed(1.0),
           rotationSpeed(1.0),
           minTurningRadius(0.0),
           multiplierForward(1),
           multiplierBackward(1),
           multiplierLateral(1),
           multiplierForwardTurn(1),
           multiplierBackwardTurn(1),
           multiplierPointTurn(1),
           multiplierLateralCurve(1) {
    }
    
    Mobility(double speed, double turning_speed, double min_turning_radius, 
             unsigned int mult_forward=0, 
             unsigned int mult_backward=0, 
             unsigned int mult_lateral=0, 
             unsigned int mult_forward_turn=0, 
             unsigned int mult_backward_turn=0, 
             unsigned int mult_pointturn=0,
             unsigned int mult_lateral_curve=0
            ) :
            translationSpeed(speed), 
            rotationSpeed(turning_speed),
            minTurningRadius(min_turning_radius),
            multiplierForward(mult_forward), 
            multiplierBackward(mult_backward),
            multiplierLateral(mult_lateral), 
            multiplierForwardTurn(mult_forward_turn), 
            multiplierBackwardTurn(mult_backward_turn),
            multiplierPointTurn(mult_pointturn),
            multiplierLateralCurve(mult_lateral_curve){
    }
};


}
