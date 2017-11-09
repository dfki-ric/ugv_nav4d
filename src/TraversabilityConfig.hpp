#pragma once

namespace ugv_nav4d
{

/**Different metrics can be used to factor in the slope of a motion  */
enum SlopeMetric
{
    
    AVG_SLOPE,
    MAX_SLOPE,
    TRIANGLE_SLOPE,
    NONE
};

enum HeuristicType
{
    HEURISTIC_2D,
    HEURISTIC_3D
};
    
class TraversabilityConfig
{
public:
    
    TraversabilityConfig()
        : maxStepHeight(0.05)
        , maxSlope(0.5)
        , inclineLimittingMinSlope(0.2)
        , inclineLimittingLimit(0.1)
        , costFunctionDist(0.4)
        , minTraversablePercentage(0.8)
        , robotHeight(0)
        , robotSizeX(0)
        , robotSizeY(0)
        , distToGround(0)
        , slopeMetricScale(1.0)
        , slopeMetric(NONE)
        , heuristicType(HEURISTIC_2D)
        , parallelismEnabled(true)
        , gridResolution(0.0)
        , ignoreCollisions(false)
        , initialPatchVariance(0.01 * 0.01)
        , allowForwardDownhill(false)
        
    {};
    
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
    
    /**Objects within a corridor of width costFunctionDist around a trajectory
     * will influence the cost function. */
    double costFunctionDist;
    
    /**
     * This value controls, how unknown patches are detected.
     * If only a certain percentage of MSL patches are present,
     * on the surface of a traversability patch, it is rated
     * an unknown patch.
     * */
    double minTraversablePercentage;
    
    double robotHeight;
    double robotSizeX;
    double robotSizeY;
    /* Distance from body frame to ground
     * start and goal position are expected in body frame
     */
    double distToGround;
    double slopeMetricScale;
    SlopeMetric slopeMetric;//which metric to use to factor in the slope of a motion
    HeuristicType heuristicType;
    bool parallelismEnabled; //if true openMP will be used to parallelize the planning
    double gridResolution;
    bool ignoreCollisions; //Disable all obstacle checks
    double initialPatchVariance; //the variance that is set for initially added patches.
    
    //if true the robot is allowed to drive downhill forward, otherwise it has to drive downhill backwards
    bool allowForwardDownhill;
};


}
