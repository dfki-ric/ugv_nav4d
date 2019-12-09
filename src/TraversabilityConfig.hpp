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


struct PlannerConfig
{
    /** The initial epsilon for the internal ARA* algorithm.
     *  See SBPL documentation for an explantion of this value*/
    double initialEpsilon = 20.0;
    /** The epsilon step size for the internal ARA* algoritm.
     * See SBPL documentation for an explantion of this value*/
    double epsilonSteps = 2.0;
    /** Number of threads to use during planning */
    unsigned numThreads = 1;
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
        , gridResolution(0.0)
        , initialPatchVariance(0.01 * 0.01)
        , allowForwardDownhill(true)
        , enableInclineLimitting(false)        
    {};
    
    /** The maximum step height that the robot can traverse.
     *  This is used during map expansion. Steps heigher than this become map boundaries */
    double maxStepHeight;
    
    /**[rad] maximum traversable slope. Above this slope no travmap entries will be generated*/
    double maxSlope; 
    
    /** ---- incline limitting -----
     * The orientation at which the path crosses the incline of the terrain is limited. The steeper the terrain, the less is the path allowed to deviate from the steepest direction. I.e. the steeper it gets, the more
     * straight the path becomes.
     * */
    double inclineLimittingMinSlope;//[rad] below this slope the robot may move freely, the incline is not limited.
    /** [rad] At maxSlope the robot's movement direction may only deviate by +-inclineLimittingLimit
     * from the direction of the steepest slope */
    double inclineLimittingLimit;
    
    /**Objects within a corridor of width costFunctionDist around a trajectory will influence the cost function. */
    double costFunctionDist;
    
    /**
     * This value controls, how unknown patches are detected.
     * If only a certain percentage of MSL patches are present,
     * on the surface of a traversability patch, it is rated
     * an unknown patch.
     * */
    double minTraversablePercentage;
    
    //dimensions of the robot bounding box.
    double robotHeight;
    double robotSizeX;
    double robotSizeY;
    
    /* Distance from body frame to ground
     * start and goal position are expected in body frame
     */
    double distToGround;
    
    /** Defines how strong the slope is factored into the 
     *  motion cost.*/
    double slopeMetricScale;
    
    /** which metric to use to factor in the slope of a motion */
    SlopeMetric slopeMetric;
    
    double gridResolution;
    
    /** The variance that "initial patches" should have.
     *  @see Planner::setInitialPatch() */
    double initialPatchVariance; 
    
    /**if true the robot is allowed to drive downhill forward, otherwise
     * it has to drive downhill backwards */
    bool allowForwardDownhill;
    
    /** if true, incline limitting is enabled and the robot motion is restricted when moving on steep hills. */
    bool enableInclineLimitting;
    
};


}
