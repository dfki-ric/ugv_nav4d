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
    // Defines the forward/backward speed of the system which will be assigned
    // to the trajectory.
    double mSpeed; // m/sec.
    // Used for cost calculations.
    double mTurningSpeed; // rad/sec
    // If > 0 allows to specify the minimal turning radius of the system in meter.
    // Without this not valid curves may be created.
    double mMinTurningRadius; 
    // Multipliers: Allows to define multipliers for each movement (used by SBPL).
    // If a multiplier is set to 0, this movement type will be deactivated.
    unsigned int mMultiplierForward;
    unsigned int mMultiplierBackward;
    unsigned int mMultiplierLateral;
    unsigned int mMultiplierForwardTurn;
    unsigned int mMultiplierBackwardTurn;
    unsigned int mMultiplierPointTurn;
    // Inspect curves are lateral curves circling an object.
    unsigned int mMultiplierLateralCurve;
    
    
    Mobility() : 
           mSpeed(0.0),
           mTurningSpeed(0.0),
           mMinTurningRadius(0.0),
           mMultiplierForward(0),
           mMultiplierBackward(0),
           mMultiplierLateral(0),
           mMultiplierForwardTurn(0),
           mMultiplierBackwardTurn(0),
           mMultiplierPointTurn(0),
           mMultiplierLateralCurve(0) {
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
            mSpeed(speed), 
            mTurningSpeed(turning_speed),
            mMinTurningRadius(min_turning_radius),
            mMultiplierForward(mult_forward), 
            mMultiplierBackward(mult_backward),
            mMultiplierLateral(mult_lateral), 
            mMultiplierForwardTurn(mult_forward_turn), 
            mMultiplierBackwardTurn(mult_backward_turn),
            mMultiplierPointTurn(mult_pointturn),
            mMultiplierLateralCurve(mult_lateral_curve){
    }
};


}
