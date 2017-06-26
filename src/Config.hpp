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


}