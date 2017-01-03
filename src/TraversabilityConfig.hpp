#pragma once

namespace ugv_nav4d
{

class TraversabilityConfig
{
public:
    TraversabilityConfig(): maxStepHeight(0), maxSlope(0), robotHeight(0), robotSizeX(0),    gridResolution(0.0) {};
    
    double maxStepHeight;
    double maxSlope; /**[rad] maximum traversable slope. Above this slope no travmap entries will be generated*/
    
    /** ---- incline limitting -----
     * The orientation at which the path crosses the incline of the terrain is limited. The steeper the terrain, the less is the path allowed to deviate from the steepest direction. I.e. the steeper it gets, the more
     * straight the path becomes.
     * */
    double inclineLimittingMinSlope;//[rad] below this slope the robot may move freely, the incline is not limited.
    /** [rad] At maxSlope the robot's movement direction may only deviate by +-inclineLimittingLimit
     * from the direction of the steepest slope */
    double inclineLimittingLimit;
    
    double robotHeight;
    double robotSizeX;
    double robotSizeY;
    double slopeMetricScale;
    double gridResolution;
};

}