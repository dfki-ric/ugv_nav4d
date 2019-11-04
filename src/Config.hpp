#pragma once

namespace ugv_nav4d{

struct CostFunctionParameters
{
    CostFunctionParameters() : 
        distToGoalFactor(1.0),
        distFromStartFactor(1.0),
        explorableFactor(1.0)
    {
    }
    /** How important is the distance from the node to the goal position  */
    double distToGoalFactor;
    /** How important is the distance from the start node */
    double distFromStartFactor;
    
    //FIXME when is explorableFactor used?
    /**How important is the size of the explorable area around the frontier node */
    double explorableFactor;
};


//ATTENTION Mobility is copied from motion_planning_libraries to avoid the dependency. Coping a simple struct is better than getting all the dependencies...
/**
 * Describes the mobility of the system.
 * If a speed has been defined it will be used to define the 
 * forward/backward speed for the trajectory.
 * The minimal turning radius is used to create valid curves.
 * The multipliers are used by SBPL during planning. In addition
 * if a multiplier is set to 0 this movement will be deactivated.
 */
struct Mobility {
    // Defines the forward/backward speed of the robot. Is used for cost calculations
    double translationSpeed; // m/sec.
    // Defines the rotational speed of the robot. Is used for cost calculations.
    double rotationSpeed; // rad/sec
    // If > 0 allows to specify the minimal turning radius of the system in meter.
    // Without this not valid curves may be created.
    double minTurningRadius; 
    // Multipliers: Allows to define multipliers for each movement (used by SBPL).
    // If a multiplier is set to 0, this movement type will be deactivated.
    unsigned int multiplierForward;
    unsigned int multiplierBackward;
    unsigned int multiplierLateral;
    unsigned int multiplierForwardTurn;
    unsigned int multiplierBackwardTurn;
    unsigned int multiplierPointTurn;
    // Inspect curves are lateral curves circling an object.
    unsigned int multiplierLateralCurve;
    
    
    Mobility() : 
           translationSpeed(0.0),
           rotationSpeed(0.0),
           minTurningRadius(0.0),
           multiplierForward(0),
           multiplierBackward(0),
           multiplierLateral(0),
           multiplierForwardTurn(0),
           multiplierBackwardTurn(0),
           multiplierPointTurn(0),
           multiplierLateralCurve(0) {
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
