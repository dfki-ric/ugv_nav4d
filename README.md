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

The heuristic h(a,b) between two cells a and b is the length of the shortest path from a to b. The shortest path is calculated without taking any of the following into account:
- the robot dimensions
- collision checks
- steepness of the terrain
- motion primitives
- motion restrictions of the robot

I.e. it is the length of the path that the robot would be able to follow if it was infinitesimal small and could change direction instantly. 

The heuristic is computed beforehand for all nodes of the map. This might be a problem for very large maps but for the current maps it is fine. Changing the code to on-demand heuristic calculation is possible. It was not done because it was not needed (fast enough) at the time of writing.



#### Motion Primitives
Configuration and how the result looks etc.






