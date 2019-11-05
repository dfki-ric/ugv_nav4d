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
How does the planning work, generally.
* sbpl based
* Ara*


How does obstacle cheking work. Obstacle Map etc.
How is cost calculated.

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





`Motion` adds the cost (and some other boilerplate that makes planning easier) and samples the splines using the planning grid resolution.

##### Motion Cost Calculation

- was machen CostFunctionParameters?
- baseCost erklären
- erklären wie die baseCost erweitert wird beim planen




