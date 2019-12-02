ugv_nav4d
=============
A 4D (X,Y,Z, Theta) Planner for unmaned ground vehicles (UGVs).


### Installation

#### Dependencies
See the `manifest.xml` for an up to date list of dependencies.

#### Compiling inside a ROCK environment

The easiest way to build and install this package is to use Rock’s build system. See http://rock-robotics.org/documentation/installation.html for addional information.


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

The visualizer of the `TraversabilityMap3D` uses color coding to indicate the different patch types:
- Green: Traversable
- Red: Not traversable.
- Blue: Traversable frontier. End of explored map, border to unknown.
- Magenta: Unknown. This is a virtual patch that serves as boundary for algorithms. This patch does not exist in reality. I.e. there is no data in the MLS to support the existence of this patch.
- Cyan: Hole. This is part of the map specification but is not generated by ugv_nav4d. It might be used elsewhere but the planner cannot handle it.
- Yellow: Unset. This is the starting state of a new patch. It should not be visible in a fully explored map. If you see a yellow patch after map expansion is done, you have found a bug in the `TraversabilityMapGenerator` and should investigate.


#### Obstacle Checking
To ensure that the robot can traverse a certain area, obstacle checks have to be done.
If we would have infinite resources we could just collide the robot model with the MLS for every possible state and see if it collides or not.
But since we have very limitted resources we cannot do that. Instead there are several places where obstacle checks occure:

##### 1. Obstacle Checks done during Expansion of the `TraversabilityMap3D`
All obstacle checks in this phase are done using the rotation invariant bounding box of the robot. This is an axis aligned bounding box with side length `a` where `a = min(config.robotSizeX, config.robotSizeY)`. All obstacle checks done with this bounding box underestimate. I.e. when an a patch is an obstacle we are 100% sure that it is. But if it is not we cannot be sure that it is really not.

- **Step height check**: A patch is an obstacle if the height between the patch and its neighbors is higher than the maximum step height of the robot. If the robot would stand on this patch, the neighboring patch would be inside the robots body. 
- **Slope check**: A patch is an obstacle if the slope of the patch is above the slope limit.
- **Map limit check**: A patch is an obstacle if the bounding box of the robot (again just using the smaller side length) leaves the map.
- 




#### The `ObstacleMap`
The `ObstacleMap` is a `TraversabilityMap3D` and is created by the `ObstacleMapGenerator`.
The generator shares a lot of code with the `TraversabilityMapGenerator`. It differs only in how obstacle checks are done, i.e. what patches are marked as obstacles.

Patches are marked as obstacle if there is a patch above the marked patch and below robot height. I.e. if the robot would stand on this patch, the patch above would be inside the robot.
The only robot dimension needed to generate the obstacle map is the height.

Using the obstacle map, the 3D collision test (that is done during planning) is reduced to a 2D collision test on the obstacle map. This is the only reason for the existence of the obstacle map. It reduces the 3D collison check complexity to 2D. Basically the height check is pre-computed for each patch and stored in the obstacle map. This greatly reduced runtime because it turns repeated 3D oriented-bounding-box-intersections into 2D-oriented-boundingbox-intersections which are much faster.

The `ObstacleMap` is used during planning to check if successor states are inside an obstacle or not.



##### Dumping Planner State
In case of error the `Planner` dumps its state to a file (this can be enabled using the `dumpOnError` parameter).
The state can be loaded and analyzed using the `ugv_nav4d_replay` binary. This binary loads the state and executes the planning in a controlled environment. This can be used to debug the planner. 



#### Heuristic

The heuristic h(a,b) between two cells a and b is the time it would take the robot to follow the shortest path from a to b. The shortest path is calculated ***without*** taking any of the following into account:
- the robot dimensions
- collision checks
- steepness of the terrain
- motion primitives
- motion restrictions of the robot

I.e. it is the path that the robot would be able to follow if it was infinitesimal small and could change direction instantly. 

The heuristic is computed beforehand for all nodes of the map. Changing the code to on-demand heuristic calculation is possible. It was not done because it was not needed (fast enough for our maps) at the time of writing.

SBPL expects the heuristic to be an integer. To avoid losing precision when converting to int the heuristic value is scaled by `Motion::costScaleFactor` (usually 1000) before conversion. Without the scaling small movements have no cost at all.

#### Motion Primitives
The planner uses motion primitives, a set of pre-defined small motions, to determine how the robot can move from one state to the next.
The basic shapes of the motion primites are generated by the `SbplSplineMotionPrimitives` libraray.
The library generates primitives using splines based on a few parameters in a perimeter around the robot.

To keep the number of primitives reasonable they are discretized. Their start and end positions are discretized using a 2d grid. The start and end orientations are discretized using angle segments.

The parameters for primitive generation are grouped in the `SplinePrimitivesConfig` class.

- `gridSize` - The width/height of a grid cell of the planning grid. This should be the same as the resolution of the map. Available end positions will be a multiple of this.
- `numAngles` - The number of discrete start orientations. A full set of primitives will be generated for each orientation.
- `numEndAngles` - The maximum number of end orientation. For each start orientation and each end orientation a full set of primitives will be generated. This is an upper boundry. It might not be reached.
- `destinationCircleRadius` - Radius around the robot (in cells) that primitives will be generated for.
- `cellSkipFactor` - Sparseness of the generated primitives.
- `splineOrder` Order of the generated splines.


![splines](https://git.hb.dfki.de/entern/ugv_nav4d/raw/standalone/doc/splines.gif)
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

The planner filters the primitives by `config.minTurningRadius` (i.e. all primitives that have a curvature that is larger than allowed by the minimum turning radius are ignored)

The following animation shows the same primitives as above but filtered with a `minTurningRadius` of `0.2`:
![splines](https://git.hb.dfki.de/entern/ugv_nav4d/raw/standalone/doc/splines_filtered.gif)

As you can see all sharp turns have been removed from the splines.

After filtering the splines are sampled using the planning grid resolution and the base cost for each motion is calculated. The sampled positions are later used during planning.

If your environment contains tight spots it is recommended to enable `generateBackwardMotions`. Otherwise the planner will have a hard time finding solutions to get to the correct end orientation in tight spots.


##### Motion Cost Calculation
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
The travelTime is scaled by 1000 to retain precision when converting to integer.

The `baseCost` is used for cost calculation during planning.
However since all primitives are 2-dimensional the `baseCost` is only accurat on perfectly flat terrain. To factor in the slope of the terrain the cost is scaled based on one of the following metrics:
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
This one is a littel tricky. 
We take the length of the spline and project it onto the slope between the start and end position. Then we measure the length of the projected line and use that to re-calculate the cost using the base cost formula (see above). This should give a good approximation of the real travel time needed to move up (or down) a slope.
```
heightDiff = < height difference between start and end of motion >
approxMotionLen = sqrt(motion.translationlDist^2 + heightDiff^2)
cost = calculateCost(approxMotionLen)
```

None of those metrices captures the real cost of moving up or down a slope. They have been implemented for experimentation. However those experiments have never been done (we ran out of time and there where no slopes in the final demo). Thus the performance of the metrices is unclear.









