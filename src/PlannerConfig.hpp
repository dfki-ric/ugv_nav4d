#pragma once

namespace ugv_nav4d{
/**
 * Describes the planner config of the path planner.
 */
struct PlannerConfig
{
    /** Should a computationally expensive obstacle check be done to check whether the robot bounding box
    *  is in collision with obstacles. This mode is useful for highly cluttered and tight spaced environments */      
    bool usePathStatistics = false;
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
    /** Corridor width (in meters) to constrain A* search around the 2D Dijkstra path.
     *  Set to <= 0.0 to disable corridor pruning. */
    double corridorWidth = -1.0;
    /** Maximum processor time to use (in seconds). */
    double maxTime = 5.0;
    /** Margin around the end orientation in radians. If the successor node's heading
     * is within this margin of the goal heading, it is mapped to the goal state.
     * Set to <= 0.0 to disable. */
    double goalOrientationMargin = 0.0;
    /** Margin around the goal position in meters. If the successor node's position
     * is within this margin of the goal position, it is mapped to the goal state.
     * Set to <= 0.0 to disable. */
    double goalDistanceMargin = 0.0;
    /** If true, the final path is reconstructed from the solution states using
     *  Reeds-Shepp steering (greedy shortcutting) instead of the search motion
     *  primitives. The primitive-based search itself is unchanged. Requires
     *  Mobility::minTurningRadius > 0, otherwise the primitive path is used. */
    bool useReedsSheppFinalPath = false;
    /** Sampling resolution (meters) of the Reeds-Shepp final path. If <= 0.0 a
     *  value of half the grid resolution is used. */
    double reedsSheppStepSize = 0.0;
    /** Maximum number of solution waypoints a single Reeds-Shepp shortcut may span.
     *  <= 0 means unlimited (best path quality, O(n^2) shortcut search). */
    int reedsSheppMaxShortcut = 0;
    /** Hybrid-A* style analytic goal connection: during the search, attempt to connect an
     *  expanded state directly to the goal with a single collision-free Reeds-Shepp curve.
     *  When it succeeds the search terminates immediately, skipping the expensive expansion
     *  of states around the goal heading. Requires useReedsSheppFinalPath = true (the final
     *  path reconstruction recreates the curve). */
    bool useReedsSheppGoalShot = false;
    /** Only attempt the goal shot when the (point-robot) distance to the goal is within this
     *  many meters. Keeps the shot near the goal where it pays off. */
    double reedsSheppGoalShotMaxDistance = 15.0;
    /** Maximum number of direction changes (cusps) a single accepted Reeds-Shepp curve may
     *  contain. During shortcutting a direction flip relative to the previously emitted
     *  segment counts towards the budget as well, so chains of alternating short curves are
     *  suppressed too. The goal shot only offers connections within this budget. Curves over
     *  the budget are skipped; the primitive path remains as fallback so feasibility is
     *  unaffected. < 0 disables the limit (legacy behavior: any collision-free curve). */
    int reedsSheppMaxCusps = 1;
};
}
