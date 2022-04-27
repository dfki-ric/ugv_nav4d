ugv_nav4d
=============
A 4D (X,Y,Z, Theta) Planner for unmaned ground vehicles (UGVs).


### Installation

#### Dependencies
See the `manifest.xml` for an up to date list of dependencies.

#### Compiling inside a ROCK environment
Buidling inside ROCK works as usual.
The package can be found in the backbone package set: https://git.hb.dfki.de/sw-backbone/package_sets


#### Compiling standalone

Several dependencies need to be compiled and installed using a common install prefix.
The following steps describe what needs to be done to get a working standalone version of `ugv_nav_4d`.

##### Prepare environment
Create env.sh with following content and source it:
```
export CMAKE_PREFIX_PATH=<PATH_TO_INSTALL_PREFIX>
export PKG_CONFIG_PATH=<PATH_TO_INSTALL_PREFIX>/lib/pkgconfig:<PATH_TO_INSTALL_PREFIX>/share/pkgconfig:<PATH_TO_INSTALL_PREFIX>/lib64/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=<PATH_TO_INSTALL_PREFIX>/lib:<PATH_TO_INSTALL_PREFIX>/lib64:$PKG_CONFIG_PATH
export PATH=<PATH_TO_INSTALL_PREFIX>/bin:$PATH
export VIZKIT_PLUGIN_RUBY_PATH=<PATH_TO_INSTALL_PREFIX>/lib
```

Most of the environment variables are only needed while compiling. Only *VIZKIT_PLUGIN_RUBY_PATH* needs to be exported for execution. This variable is used by vizkit3d to locate the visualization plugins.

Visualization is only required by the test guis. The planner itself can work without vizkit.


##### install base-cmake
base-cmake contains special cmake macros that are used in osgviz, vizkit3d and V3DD.

```
git clone git@github.com:rock-core/base-cmake.git
cd base-cmake
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<PATH_TO_INSTALL_PREFIX> ..
make -j install
```

##### install osgviz
```
git clone git@github.com:rock-core/gui-osgviz.git
cd gui-osgviz
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<PATH_TO_INSTALL_PREFIX> ..
make -j install
```

##### install vizkit3d
```
git clone git@github.com:envire/gui-vizkit3d.git
cd gui-vizkit3d
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<PATH_TO_INSTALL_PREFIX> ..
make -j install
```

##### install base-logging

```
git clone git@github.com:rock-core/base-logging.git
cd base-logging
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<PATH_TO_INSTALL_PREFIX> ..
make -j install
```

##### install SISL
Build SISL as shared library
```
git clone git@github.com:SINTEF-Geometry/SISL.git
cd SISL
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<PATH_TO_INSTALL_PREFIX> -DBUILD_SHARED_LIBS=On ..
make -j install
```

##### install base-types
build base-types without ruby support to avoid the ruby dependencies
```
git clone git@github.com:rock-core/base-types.git
cd base-types
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<PATH_TO_INSTALL_PREFIX> -DBINDINGS_RUBY=Off ..
make -j install
```

##### install sbpl
sbpl is used as underlying planner

```
git clone git@github.com:sbpl/sbpl.git
cd sbpl
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<PATH_TO_INSTALL_PREFIX> ..
make -j install
```

##### install sbpl_spline_primitives
```
git clone git@github.com:rock-planning/planning-sbpl_spline_primitives.git
cd sbpl_spline_primitives
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<PATH_TO_INSTALL_PREFIX> ..
make -j install
```

##### install base-numeric
```
git clone git@github.com:rock-core/base-numeric.git
cd base-numeric
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<PATH_TO_INSTALL_PREFIX> ..
make -j install
```

##### install base-boost_serialization
```
git clone git@github.com:envire/base-boost_serialization.git
cd base-boost_serialization
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<PATH_TO_INSTALL_PREFIX> ..
make -j install
```

##### install vizkit3d_debug_drawings
Outside of rock ports dont exists, therefore disable port support.
```
git clone git@github.com:rock-gui/gui-vizkit3d_debug_drawings.git
cd gui-vizkit3d_debug_drawings
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<PATH_TO_INSTALL_PREFIX> -DWITH_PORTS=OFF -DROCK_TEST_ENABLED=ON ..
make -j install
```

##### Install pcl
At the time of writing there seems to a bug in pcl versions above commit 0d43316f62a5142d735db948679beb05412894ff that causes a segfault. Mostly likely the bug is not in pcl but in how we use it. However this has not been investigated further as the specified commit works.
It might very well be that a newer version works for you. Feel free to try!
```
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
git checkout 0d43316f62a5142d735db948679beb05412894ff
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<PATH_TO_INSTALL_PREFIX> ..
make -j install
```

##### install slam-maps
slam-maps has to be built after all the gui stuff, otherwise it will fallback to
building without vizkit plugins.

the current pcl version needs at least c++14.
patch the cmake file and set the CMAKE_CXX_STANDARD_REQUIRED to 14.

```
git clone git@github.com:envire/slam-maps.git
cd slam-maps
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<PATH_TO_INSTALL_PREFIX> ..
make -j install
```

##### install ugv_nav4d
Finally install ugv_nav4d and switch to standalone branch
```
git clone git@git.hb.dfki.de:entern/ugv_nav4d.git
cd ugv_nav4d
git checkout standalone
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<PATH_TO_INSTALL_PREFIX> -DWITH_PORTS=OFF -DROCK_TEST_ENABLED=ON ..
make -j install
```

### Planning
The planner is based on SBPL (http://www.sbpl.net/). I.e. it uses the SBPLs ARA* planner to plan in a custom environment.

TODO parallellism

#### Environment
SBPL internally uses states (identified by an id only) and associated costs (a unitless integer). The state ids reference states of an environment. That environment has to be defined by the user.
The planner contains the `EnvironmentXYZTheta`. This environment implements all interfaces needed by SBPL to enable ARA* planning (other planning algorithms might require additional interfaces).


A state in this environment consists of the position on the map (xyz) and the orientation of the robot (theta), hence the name. 

The mapping `idToHash` in `EnvironmentXYZTheta` maps the SBPL state ids to instances of `Hash` (our internal representation of a state).
```
struct Hash
{
    XYZNode *node;
    ThetaNode *thetaNode;
};
```

The `Hash` represents a complete planner state. It consists of an `XYZNode` and a `ThetaNode`. The `XYZNode` represents a position on the traversability map while the `ThetaNode` holds the discretized orientation. Together they form a planner state. One `XYZNode` can be part of several states (with different `ThetaNodes`). However, a new `ThetaNode` is created for each state.
The `XYZNode` contains a map to all `ThetaNodes` that it has been associated with during planning. 

All `XYZNodes` are part of the `searchGrid`.
An `XYZNode` is always created from (and corresponds to) a `TravGenNode` and shares the `TravGenNodes` index. I.e. the `XYZNodes` position in the `searchGrid` is the same as the `TravGenNodes` position on the `traversabilityMapGenerator::travMap`. For easy access the `XYZNode` contains a pointer to the corresponding `TravGenNode`.

The `searchGrid` keeps track of the internal state while planning. It contains an `XYZNode` for every grid cell that the planner has already visited.
The `searchGrid` is a `TraversabilityMap` but the traversability information is not used (and never set).
The `TraversabilityMap` was chosen because it enables O(n) (n <= maximum number of layers in the map) lookup of nodes based on their xyz position. The choice was made due to time constraints (the map was there and it worked and there was no time). Semantically the usage of a `TraversabilityMap` to store the `XYZNodes`is wrong and it should be changed (e.g. to a hashmap), but it works and thus was never changed.

#### The `TraversabilityMap`
In addion to the `searchGrid` the environment has access to a `TraversabilityMap3D` (accessed through the `travGen` attribute).

A `TraversabilityMap3D` is generated from an MLS and separates the MLS into traversable, non-traversabel and unknown terrain. Addtionally it contains some meta data (e.g. slope of the patch, supporting plane, etc.).
The map is generated by the `TraversabilityMapGenerator` based on a set of rules.

The `TraversabilityMap3D` has to be fully expanded (i.e. generated from the MLS) before planning. In theory it could be expanded on the fly during planning and the code is prepared to do that, however this was never tested and there is a good chance that on-the-fly expansion will trigger bugs (especially if parallelism is enabled).

The planner uses the `TraversabilityMap3D` to find valid successor states during planning. I.e. states that the robot can traverse to from a given state using the given motion primitives. Metadata stored in the map (e.g. slope) is also used during planning to calculate costs.

#### Color Codes
![color_codes](https://git.hb.dfki.de/entern/ugv_nav4d/raw/master/doc/color_codes.png)

The visualizer of the `TraversabilityMap3D` uses color coding to indicate the different patch types:
- **Traversable**: The robot can stand (with its center) on this patch in at least one orientation without hitting an obstacle.
- **Not Traversable**: There is no way that the robot can stand (with its center) on this patch.
- **Frontier**: Borders to the end of the map. Should be traversable (I am not 100% sure about this. check the code!)
- **Unknown**: This is a virtual patch that serves as boundary for algorithms. This patch does not exist in reality. Patches also become unknown if there is not enough support in the MLS to be sure that a patch exists in this location.
- **Hole**: This is part of the map specification but is not used by ugv_nav4d. It might be used elsewhere but the planner cannot handle it.
- **Unset**: This is the starting state of a new patch. It should not be visible in a fully explored map. If you see a yellow patch after map expansion is done, you have found a bug in the `TraversabilityMapGenerator` and should investigate.


![2022-03-02_10-45](/uploads/68a0caf1d9cb6d5923d552a0f8502954/2022-03-02_10-45.png)

#### Obstacle Checking
To ensure that the robot can traverse a certain area, obstacle checks have to be done.
If we would have infinite resources we could just collide the robot model with the MLS for every possible state and see if it collides or not.
But since we have very limitted resources we cannot do that. Instead there are several obstacle checking phases:

##### 1. Obstacle Checks done during Expansion of the `TraversabilityMap3D`
All obstacle checks in this phase are done using the rotation invariant bounding box of the robot. This is an axis aligned bounding box with side length `min(config.robotSizeX, config.robotSizeY)`. With this check when an a patch is an obstacle we are 100% sure that it is. But if it is not we cannot be sure that it is not. This greatly reduces map size and planning time without costing too much as checks are only done once per patch.
It also means that a full 3D oriented bounding box check is still necessary during planning to factor in different side lengths and the orientation of the robot.

- **Step height check**: A patch is an obstacle if the height between the patch and its neighbors is higher than the maximum step height of the robot. If the robot would stand on this patch, the neighboring patch would be inside the robots body. 
- **Slope check**: A patch is an obstacle if the slope of the patch is above the slope limit.
- **Map limit check**: A patch is an obstacle if the bounding box of the robot (again just using the smaller side length) leaves the map (maximum possible map, not currently known map).

I.e. this step marks patches in the `TraversabilityMap3D` as obstacle if the robot would touch an obstacle when standing (centered) on this patch, or when the slope is too steep.



##### 2. The `ObstacleMap`
The `ObstacleMap` is a `TraversabilityMap3D` and is created by the `ObstacleMapGenerator`.
The generator shares a lot of code with the `TraversabilityMapGenerator`. It differs only in how obstacle checks are done, i.e. what patches are marked as obstacles.

Patches are marked as obstacle if there is a patch above the marked patch and below robot height. I.e. if the robot would stand on this patch, the patch above would be inside the robot.
This is a tiny but important difference. It means that, if the robot is on this patch with even a tiny bit of its body, it would touch an obstacle.

Using the obstacle map, the 3D collision test (that is necessary during planning) is reduced to a 2D oriented bounding box test. This is the only reason for the existence of the obstacle map. It reduces the complexity of the 3D collison check to 2D. Basically the height check is pre-computed for each patch and stored in the obstacle map. 

The `ObstacleMap` is used during planning to check if the robot would hit an obstacle when moving to a certain state. 


#### Heuristic
ARA* is a real-timeish version of A*. Thus it needs a heuristic.

The heuristic h(a,b) between two cells a and b is the time it would take the robot to follow the shortest path from a to b on the TraversabilityMap3D. The shortest path is calculated ***without*** taking any of the following into account:
- the robot dimensions and orientation (The minimum bounding box has already been checked while expanding the map)
- collision checks on the ObstacleMap
- steepness of the terrain (i.e. as long as a patch has a steepness below the limit the costs are the same)
- motion primitives
- motion restrictions of the robot

I.e. it is the path that the robot would be able to follow if it was infinitesimal small and could change direction instantly. 

The heuristic is computed beforehand for all nodes of the map. Changing the code to on-demand heuristic should be possible. It was not done because it was not needed (fast enough for our maps) at the time of writing.

SBPL expects the heuristic to be an integer. To avoid losing precision when converting to int the heuristic value is scaled by `Motion::costScaleFactor` (usually 1000) before conversion. Without the scaling small movements have no cost at all.


#### Motion Primitives
The planner uses motion primitives, a set of pre-defined small motions, to determine how the robot can move from one state to the next.  The primitives are classified into four categories, namely:

**Forward Primitive**
**Backward Primitive**
**Lateral Primitive**
**Point-Turn Primitive**

All four primitive motion types are considered as valid motions. It needs to be mentioned that the point-turn primitives are a special case because they do not use a spline. On the contrary, forward, backward, and lateral motion primitives are splines. 

The basic shapes of the motion primites are generated by the `SbplSplineMotionPrimitives` library.
The library generates primitives using splines based on a few parameters in a perimeter around the robot.

The parameters for primitive generation are grouped in the `SplinePrimitivesConfig` class.

- `gridSize` - The width/height of a grid cell of the planning grid. This should be the same as the resolution of the map. Available end positions will be a multiple of this.
- `numAngles` - The number of discrete start orientations. A full set of primitives will be generated for each orientation.
- `numEndAngles` - The maximum number of end orientation. For each start orientation and each end orientation a full set of primitives will be generated. This is an upper boundry. It might not be reached.
- `destinationCircleRadius` - Radius around the robot (in cells) that primitives will be generated for.
- `cellSkipFactor` - Sparseness of the generated primitives.
- `splineOrder` Order of the generated splines.


Based on the value of the parameter `destinationCircleRadius` a number of discrete destination points are generated on concentric circles. The parameter `CellSkipFactor` decides the interval between each two consecutive concentric circles. Before generating a motion primitive, each destination cell is scaled via multiplication with the `gridSize`. A unique motion primitives is generated for each start angle to each destination cell shown in picture below, from `(0,0)` to that cell for each end angle. The number of start angles and end angles is decided basd on the parameters `numAngles` and `numEndAngles` respectively.

![GetImage](/uploads/0fc6280c33594bebff819888c7b58404/GetImage.png)

To keep the number of primitives reasonable they are discretized. Their start and end positions are discretized using a 2d grid. The start and end orientations are discretized using angle segments.

![splines](https://git.hb.dfki.de/entern/ugv_nav4d/raw/master/doc/splines.gif)
This animation shows all splines generated by the following configuration (each frame shows the primitives for one start orientation). 
```
config.gridSize = 0.1;
config.numAngles = 24;
config.numEndAngles = 12;
config.destinationCircleRadius = 5;
config.cellSkipFactor = 1.0;
config.generatePointTurnMotions = false;
config.generateLateralMotions = false;
config.generateBackwardMotions = false;
config.splineOrder = 4;
```
##### Default Parameters
| Parameter | Type |Description | Recommented Value |
|-----------------|:-------------|:-------------|:-------------|
| dumpOnError | bool  | Refer to the section  `Dumping Planner State`  | Release: 0, Debug: 1  |
| dumpOnSuccess    | bool         | Refer to the section `Dumping Planner State`  | Release: 0, Debug: 1 |
| initialPatchRadius     | double        | Radius of initial patch of point cloud points to kick-start the planner  | 3.0 |

##### Mobility Configuration Parameters
| Parameter | Type |Description |Recommented Value |
|-----------------|:-------------|:-------------|:-------------|
| translationSpeed | double  | Linear velocity of the resulting path trajectory  | 1.0 |
| rotationSpeed    | double         | Angular velocity of the resulting path trajectory  | 1.0 |
| minTurningRadius     | double        | The minimum turning radius corresponds to the maximum curvature. This parameter therefore helps to select all motion primitives which have curvature less than the maximum curvature. A higher value means less cap for the maximum curvature resulting in linear primitives. A small value will result in more curvy primitives. Note that a smaller value results in more primitives selected which requires more computation power for the planning. For details see section `Minimum Turning Radius`  | 0.1 |
| multiplierForward | int  | Cost multiplier for the forward motion primitives  | 1 |
| multiplierBackward    | int         | Cost multiplier for the backward motion primitives  | 2 |
| multiplierLateral     | int        | Cost multiplier for the lateral motion primitives  | 2 |
| multiplierForwardTurn | int  | Cost multiplier for the forward turn motion primitives  | 1 |
| multiplierBackwardTurn    | int         | Cost multiplier for the backward turn motion primitives  | 2 |
| multiplierPointTurn     | int        | Cost multiplier for the point turn motion primitives  | 1 |
| multiplierLateralCurve | int  | Cost multiplier for the lateral curve motion primitives  | 2 |
| searchRadius    | double         |   | 1.0 |
| searchProgressSteps     | double        |   | 0.1 |
| maxMotionCurveLength | double  | The maximum curve length of the selected motion primitives. Small value results in small primitives and a large value results in longer primitives. During testing, it was observed that the planner has a hard time in finding a solution if the value of this parameter is set < 0.6  | 1.3 |

##### Planner Configuration Parameters
| Parameter | Type |Description | Recommented Value |
|-----------------|:-------------|:-------------|:-------------|
| initialEpsilon | int  | The planner uses ARA* planner. It finds a sub-optimal solution and then repairs the initial solution by using reducing the epsilon by the parameter `epsilonSteps`. An optimal solution means epsilon is equal to 1, where `solution = epsilon x optimal_solution`   | 36  |
| epsilonSteps    | int         | The steps in the epsilon during planning and repairing of the initial sub-optimal solution  | 6 |
| numThreads     | int        | A limit on the threads allocated for the planner during planning.  | 8|

##### Primitives Configuration Parameters
| Parameter | Type |Description | Recommented Value |
|-----------------|:-------------|:-------------|:-------------|
| gridSize | double  | The size of the traversability map grid  | Use case specific |
| numAngles    | int         | number of discrete start angles angles. A full set of primitives will be generated for each start angle (has to be even number). For best performance, the value should be a multiple of 4. The multiple of 4 means that correct lateral motion primitives are generated. The reason for this is that the lateral motions are seen as 90 degree rotated motions from any given start angle. For this, the start angle is divided by 4 (because of 4 quadrants) and the result is subracted from the start angle  | 40 |
| numEndAngles     | int        | The number of end angles for each destination cell. The value has to be <= numAngles/2 and odd | 9|
| destinationCircleRadius     | int        | See section `Motion Primitives`  | 6 |
| cellSkipFactor     | int        | See section `Motion Primitives`. It is best to leave this value at >= 1 in integer steps. A value of less than 1 results in duplicate destination cells and do not add any value to the planning  | 1 |
| splineOrder     | int        | The value has to be >= 3  | 3.0 |
| generateForwardMotions     | bool        | Generate forward motion primitives. Forward primitives are for the case where destination cell x-coordinate > 0.1 |  Use case specific |
| generateBackwardMotions     | bool        | Generate backward motion primitives. Backward primitives are for the case where destination cell x-coordinate < -0.1  |  Use case specific |
| generateLateralMotions     | bool        | Generate lateral motion primitives. Lateral primitives are for the case where the destination cell x-coordinate > -0.1 and destination cell x-coordinate < 0.1  |  Use case specific|
| generatePointTurnMotions     | bool        | Generate point-turn motion primitives. A point-turn is generated for each start angle to each end angle  |  Use case specific |

##### Traversability Configuration Parameters
| Parameter | Type |Description | Recommented Value |
|-----------------|:-------------|:-------------|:-------------|
| maxStepHeight | double  | The maximum step height that the robot can traverse. This is used during map expansion. Steps heigher than this become map boundaries   | Use case specific  |
| maxSlope    | double         | Maximum traversable slope above which no travmap entries will be generated | 0.45 |
| inclineLimittingMinSlope     | double        |   | 0.2 |
| inclineLimittingLimit | double  |  | 0.1 |
| costFunctionDist    | double         | Objects within a corridor of width costFunctionDist around a trajectory will influence the cost function. A higher value results in large computation cost requirements for the planning.  | 0.0 |
| minTraversablePercentage     | double        | This value controls, how unknown patches are detected. If only a certain percentage of MSL patches are present on the surface of a traversability patch, it is rated as an unknown patch.    | 0.4 |
| robotHeight | double  |  | Use case specific  |
| robotSizeX    | double         | The length of the robot along x-axis   | Use case specific |
| robotSizeY     | double        | The length of the robot along y-axis| Use case specific |
| distToGround | double  | Distance from body frame to ground. Start and goal position are expected in body frame  | Use case specific  |
| slopeMetricScale    | double         |   | 1.0 |
| slopeMetric     | double        | The slope metric used to adjust the cost of a motion | Use case specific |
| gridResolution | double  | The resolution of the traversability grid map  | Use case specific |
| initialPatchVariance    | double         |   | 0.0001|
| allowForwardDownhill     | bool        |   | true |
| enableInclineLimitting | bool  |  | false  |

##### Motion Primitive Filtering

###### Minimum Turning Radius
The planner filters the primitives by `minTurningRadius` (i.e. all primitives that have a curvature that is larger than allowed by the minimum turning radius are ignored)

The following animation shows the same primitives as above but filtered with a `minTurningRadius` of `0.2`:
![splines](https://git.hb.dfki.de/entern/ugv_nav4d/raw/master/doc/splines_filtered.gif)

As you can see all sharp turns have been removed from the splines.

After filtering the splines are sampled using the planning grid resolution and the base cost for each motion is calculated. The sampled positions are later used during planning.

If your environment contains tight spots it is recommended to enable `generateBackwardMotions`. Otherwise the planner will have a hard time finding solutions to get to the correct end orientation in tight spots.

###### Maximum Curve Length
You can filter the primitives using the parameter `maxMotionCurveLength`. All primitives which have a curve length less than the `maxMotionCurveLength` will be selected.

The figure below shows the complete set of discritized splines, **without any curve length filter applied**, generated for the start angle of 0 radians. The green color signifies discritized primitives for forward motion, magenta for backward motion, and orange for lateral motion. Please note that the point-turns do not have a spline and therefore are not visible in the image below.

![GetImage__1_](/uploads/177ff981282ad2d84f33b937ddd3b67c/GetImage__1_.png)

The figure below shows the same set of discritized splines for start angle of 0 radians but with a `maxMotionCurveLength` of 1.2.

![GetImage__2_](/uploads/54396e944d35c29e6a39da613b2ade36/GetImage__2_.png)

To get an idea about different geometric lengths of the primitives see the picture below. The picture shows only forward and backward motion primitives. Each arrow represents the end of a unqiue primitive. As mentioned earlier, the end points of the primitives are the destination cells scaled by the grid size of the traversability map. 

![GetImage__3_](/uploads/e6501fee1be56fe272a5b03fa539ab10/GetImage__3_.png)

##### Motions
Each slected motion primitive is converted into a motion. A motion is a discritized motion primitive. In the planning phase, each discrete step of the motion is used to perform traversability and obstacle checks. You can find details on the motions in the class preComputedMotions.


##### Motion Base Cost Calculation
Upon motion generation every motion is asigned a base cost.
I.e. the cost that would arise when the robot would follow that motion on a horizontal flat surface.
Factors for steepness and other penalties might be factored later during planning on a case to case basis.

The base cost for each motion is calculated as follows:
```
translationDist = < distance that the robot has to travel while following the spline >
rotationDist    = < amount that the robot has to turn while following the spline >
translationTime = translationalDist / translationVelocity
rotationTime    = rotationDist / angularVelocity
travelTime      = max(rotationTime, translationTime)
costMultiplier  = < the configured multiplier for this particular motion type >
baseCost        = int(ceil(travelTime * 1000 * costMultiplier))
```
The travelTime is scaled by 1000 to retain three digits of precision when converting to integer.

##### Motion Cost Scaling

Since all primitives are 2-dimensional the `baseCost` is only accurat on perfectly flat terrain. To factor in the slope of the terrain the cost is scaled based on one of the following metrics during planning.

###### SlopeMetric::NONE
```
cost = motion.baseCost;
```
###### SlopeMetric::AVG_SLOPE
```
slopeFactor = < avg slope under spline> * config.slopeMetricScale;
cost = motion.baseCost + motion.baseCost * slopeFactor;
```
###### SlopeMetric::MAX_SLOPE
```
slopeFactor = < max slope under spline> * config.slopeMetricScale;
cost = motion.baseCost + motion.baseCost * slopeFactor;
```
###### SlopeMetric::TRIANGLE_SLOPE
This one is a little tricky. 
We take the length of the spline and project it onto the slope between the start and end position. Then we measure the length of the projected line and use that to re-calculate the cost using the base cost formula (see above). This should give a good approximation of the real travel time needed to move up (or down) a slope.
```
heightDiff = < height difference between start and end of motion >
approxMotionLen = sqrt(motion.translationlDist^2 + heightDiff^2)
cost = calculateCost(approxMotionLen)
```

None of those metrices captures the real cost of moving up or down a slope. They have been implemented for experimentation. However those experiments have never been done (we ran out of time and there where no slopes in the final demo). Thus the performance of the metrices is unclear.


#### Dumping Planner State
In case of error the `Planner` dumps its state to a file (this can be enabled using the `dumpOnError` parameter).
The state can be loaded and analyzed using the `ugv_nav4d_replay` binary. This binary loads the state and executes the planning in a controlled environment. This can be used to debug the planner. 


#### User Interfaces
Two user interfaces can be found in `src/gui`. They are intended for testing and debugging.

##### PlannerGui
The `PlannerGui` is the main testing gui. It is designed to experiment with differen planner parameters on a static map.
It can load point clouds from ply or serialized mls maps. 
A left click sets the start location, a right click sets the end location.

In addition the `PlannerGui` can also be used to load and analyze planner dumps.


##### FrontierTestGui
This gui was developed to test and debug the `AreaExplorer`.
It allows for experimentation with the different parameters of the area explorer.
A left click sets the robot position, a right click sets the goal position.
