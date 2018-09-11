ugv_nav4d
=============
A 4D (X,Y,Z, Theta) Planner for UGVs


#### Compiling standalone

Several dependencies need to be compiled and installed using a common install prefix.

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


##### install motion_planning_libraries
```
git clone git@github.com:entern2017/planning-motion_planning_libraries.git
cd gui-osgviz-osgviz
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<PATH_TO_INSTALL_PREFIX> ..
make -j install
```
##### install vizkit3d
```
git clone git@github.com:envire/gui-vizkit3d.git
cd gui-vizkit3d
git checkout osgviz
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=<PATH_TO_INSTALL_PREFIX> ..
make -j install
```
