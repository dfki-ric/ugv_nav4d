#pragma once

namespace ugv_nav4d{
/**
 * Describes the planner config of the path planner.
 */
struct PlannerConfig
{
    /** Search only until the first solution and then stop planning
     *  See SBPL documentation for an explantion of this value*/
    bool searchUntilFirstSolution = false;
    /** The initial epsilon for the internal ARA* algorithm.
     *  See SBPL documentation for an explantion of this value*/
    double initialEpsilon = 20.0;
    /** The epsilon step size for the internal ARA* algoritm.
     * See SBPL documentation for an explantion of this value*/
    double epsilonSteps = 2.0;
    /** Number of threads to use during planning */
    unsigned numThreads = 1;
};
}
