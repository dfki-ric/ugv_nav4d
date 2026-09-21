# UGV Nav4D Parameter Tuning Guide

This document describes the function of each configuration parameter in `gui/config/parameters.yaml` and provides guidance on how to tune them to achieve optimal performance, path quality, and planner reliability.

---

## 1. `splineConfig` (Motion Primitive Generation)

These parameters control the discretization and motion types used to generate spline-based motion primitives during search.

| Parameter | Recommended Range | Description | Tuning Impact |
| :--- | :--- | :--- | :--- |
| **`gridSize`** | `0.25` - `1.0` | Resolution of the search grid in meters. | **Smaller value:** Smoother trajectories, but search space grows quadratically, leading to slower planning times.<br>**Larger value:** Faster planning, but path might look jagged and ignore narrow passages. |
| **`numAngles`** | `16` - `48` | Number of discrete angles used to represent robot heading ($360^\circ$ circle). | Must be reasonably high (e.g. 32 or 42) for smooth spline connections. Increasing this increases search space size and precomputation time. |
| **`numEndAngles`** | `8` - `24` | Number of goal-reaching angles analyzed for destination connections. | Usually set to $\approx \text{numAngles} / 2$. |
| **`destinationCircleRadius`** | `5` - `15` | Radius (in grid cells) within which target-oriented primitive splines are evaluated. | Higher value allows smoother, longer curves but increases motion generation time. |
| **`cellSkipFactor`** | `1.0` | Factor to skip cells during search. | Best kept at `1.0` for full resolution search. |
| **`generatePointTurnMotions`** | `true` / `false` | Enable/disable in-place rotation motions. | Set to `false` for Ackermann-only vehicles to prevent in-place turns. Set to `true` for skid-steer or omnidirectional robots. |
| **`generateLateralMotions`** | `true` / `false` | Enable/disable sideways/crabbing motions. | Keep `false` unless the UGV is omnidirectional / mecanum. |
| **`generateBackwardMotions`** | `true` / `false` | Enable/disable reverse driving. | Set to `true` if UGV needs to back out of tight spaces, but set to `false` to enforce forward-only driving. |
| **`generateForwardMotions`** | `true` / `false` | Enable/disable forward driving. | Always `true` under normal circumstances. |
| **`splineOrder`** | `3` - `5` | Mathematical order of the NURBS spline primitives. | `4` is standard (cubic B-splines, yielding continuous curvature). |

---

## 2. `mobilityConfig` (Cost weights and kinematic constraints)

These parameters define speed limits and penalty multipliers. They determine the shape and style of the planned path.

| Parameter | Recommended Range | Description | Tuning Impact |
| :--- | :--- | :--- | :--- |
| **`translationSpeed`** | `0.1` - `2.0` | Default translational speed of the UGV (m/s). | Used to convert path lengths into time-based heuristics and costs. |
| **`rotationSpeed`** | `0.1` - `1.5` | Default rotational speed of the UGV (rad/s). | Used to compute time-based cost for point turns. |
| **`minTurningRadius`** | `1.0` - `10.0` | Minimum turning radius constraints for curves (meters). | **Must match the physical limits of your UGV.** A value too small will generate paths the UGV cannot kinematically follow. |
| **`searchRadius`** | `1.0` | Search step radius. | Best kept at `1.0`. |
| **`searchProgressSteps`** | `0.1` | Step size for primitive generation checking. | Best kept at `0.1`. |
| **`multiplierForward`** | `1.0` | Base cost weight for forward motion. | Baseline weight (usually `1.0`). |
| **`multiplierForwardTurn`** | `1.0` - `5.0` | Penalty factor for turning while moving forward. | **Increase this** (e.g. to `4.0` or `5.0`) to penalize wiggly paths and force the planner to prefer straight lines. |
| **`multiplierBackward`** | `1.5` - `3.0` | Penalty factor for driving in reverse. | Set higher than `multiplierForward` to ensure the UGV only reverses when absolutely necessary. |
| **`multiplierBackwardTurn`** | `2.0` - `6.0` | Penalty factor for turning while reversing. | Set high to avoid complex reverse turns. |
| **`multiplierPointTurn`** | `1.5` - `5.0` | Cost multiplier for executing in-place rotations. | High values make the planner prefer long driving loops over sharp point turns. |
| **`maxMotionCurveLength`** | `5.0` - `15.0` | Maximum length in meters of a single motion primitive segment. | Keep aligned with `destinationCircleRadius`. |
| **`remove_goal_offset`** | `true` / `false` | Linearly distribute position error at the end of the trajectory to match the goal exactly. | Set to `false` if you want raw planner outputs. Set to `true` if you want the trajectory follower to target the exact clicked goal pose. |
| **`curvaturePenaltyWeight`** | `1.0` - `10.0` | Weight factor penalizing high-curvature turns. | **Increase this** (e.g., `5.0` or more) to favor wider, more sweeping turns over tight cornering. |
| **`angularCostWeight`** | `1.0` - `10.0` | Weight factor penalizing steering rate changes. | **Increase this** to make trajectories smoother and reduce rapid, high-frequency steering adjustments. |

---

## 3. `travConfig` (Traversability and Terrain Limits)

These parameters configure grid mapping, slopes, and collision checks.

| Parameter | Recommended Range | Description | Tuning Impact |
| :--- | :--- | :--- | :--- |
| **`gridResolution`** | `0.1` - `1.0` | Resolution of the underlying traversability map (meters). | Must match the grid resolution of your traversability generator. |
| **`maxSlope`** | `0.1` - `0.6` | Maximum allowed terrain incline slope. | E.g. `0.45` represents approx $24^\circ$ maximum climb limit. |
| **`maxStepHeight`** | `0.1` - `0.8` | Maximum step height / obstacle size UGV can climb (meters). | Set this based on your UGV's wheel/suspension capability. |
| **`robotSizeX`** / **`robotSizeY`** | Physical Dimensions | Length and width of the UGV bounding box (meters). | Critical for collision check safety. Too large blocks tight passage planning. |
| **`robotHeight`** | Physical Dimension | Vertical height clearance needed by the UGV. | Used to prune low overhead obstacles. |
| **`slopeMetric`** | `"AVG_SLOPE"`, `"MAX_SLOPE"`, `"NONE"` | Slope cost calculation strategy. | `"NONE"` treats all traversable cells equally. `"AVG_SLOPE"` adds cost proportional to terrain steepness. |
| **`enableInclineLimitting`** | `true` / `false` | Strictly prohibit driving on slopes exceeding configured limit. | Set `true` if UGV is top-heavy and prone to tipping. |
| **`partiallyTraversableMultiplier`** | `1.0` - `3.0` | Cost multiplier for traversing partially traversable nodes (e.g., rough gravel, tall grass). | E.g. `1.5` makes the planner prefer a longer, clean path over a shorter, rough path. |

---

## 4. `plannerConfig` (Search Algorithm and Performance)

These parameters control the SBPL ARA* search engine behavior and parallel threading.

| Parameter | Recommended Range | Description | Tuning Impact |
| :--- | :--- | :--- | :--- |
| **`initialEpsilon`** | `1.0` - `64.0` | Initial heuristic inflation factor for ARA* search. | **Larger values** (e.g. `64.0`) find a first solution extremely fast, but it will be highly suboptimal.<br>**Smaller values** (e.g. `1.0` - `3.0`) produce high-quality paths but planning takes much longer. |
| **`epsilonSteps`** | `1.0` - `5.0` | Step size by which Epsilon decreases in subsequent search iterations. | Controls how aggressively the planner refines path quality if spare time remains. |
| **`searchUntilFirstSolution`** | `true` / `false` | Stop search immediately once the first valid path is found. | **Set to `true`** for real-time reactive navigation where planning speed is critical.<br>**Set to `false`** if path optimality and smoothness are more important than execution time. |
| **`numThreads`** | `1` - `CPU count` | Number of parallel threads used to evaluate motion successor validity. | Set to match available CPU cores (typically `4` or `8`). Uses OpenMP. |
| **`usePathStatistics`** | `true` / `false` | Enable complex bounding-box collision checks. | Set to `false` for high planning speed. Set to `true` only for cluttered environments requiring precise collision clearance. |
| **`corridorWidth`** | `1.0` - `10.0` | Width of the corridor pruning envelope around the 2D Dijkstra path. | **Set to `3.0` - `5.0`** to prune nodes far from the logical path, significantly accelerating search. Set to `<= 0` to disable corridor pruning. |
| **`maxTime`** | `1.0` - `10.0` | Hard timeout limit in seconds. | Prevents the planner from taking too long if search space is vast or no solution exists. |
| **`goalOrientationMargin`** / **`goalDistanceMargin`** | Margin tolerances | Tolerances around goal pose to match end constraints. | Larger margins make the goal easier to reach, reducing planning failures near complex goals. |
