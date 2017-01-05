#pragma once

namespace ugv_nav4d
{

/**Different metrics can be used to factor in the slope of a motion  */
enum class SlopeMetric
{
    
    AVG_SLOPE,
    MAX_SLOPE,
    TRIANGLE_SLOPE,
    NONE
};

enum class HeuristicType
{
    HEURISTIC_2D,
    HEURISTIC_3D
};
    
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
    SlopeMetric slopeMetric;//which metric to use to factor in the slope of a motion
    HeuristicType heuristicType;
    double gridResolution;
};

}