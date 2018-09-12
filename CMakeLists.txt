# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
find_package(Rock)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
rock_init(ugv_nav4d 0.1)
add_definitions(-DENABLE_DEBUG_DRAWINGS)
rock_standard_layout()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")

rock_executable(ugv_nav4d_bin
    SOURCES
        src/Main.cpp
        src/PlannerGui.cpp
    HEADERS
        src/PlannerGui.h
    MOC
        src/PlannerGui.h        
   DEPS ugv_nav4d 
   DEPS_PKGCONFIG vizkit3d  maps-viz base-viz sbpl_spline_primitives-viz base-types
   DEPS_CMAKE Threads)

rock_executable(ugv_nav4d_replay
    SOURCES
        src/ReplayDump.cpp
        src/PlannerGui.cpp
    HEADERS
        src/PlannerGui.h
    MOC
        src/PlannerGui.h        
   DEPS ugv_nav4d 
   DEPS_PKGCONFIG vizkit3d  maps-viz base-viz sbpl_spline_primitives-viz base-types
   DEPS_CMAKE Threads)

rock_executable(frontier_test_bin
    SOURCES
        src/FrontierTest.cpp
        src/FrontierTestGui.cpp
    HEADERS
        src/FrontierTestGui.hpp
    MOC
        src/FrontierTestGui.hpp      
   DEPS ugv_nav4d 
   DEPS_PKGCONFIG vizkit3d  maps-viz base-viz sbpl_spline_primitives-viz base-types)
   
rock_executable(mls_clicker
    SOURCES
        src/MinimalClickGui.cpp
        src/MinimalClickMain.cpp
    HEADERS
        src/MinimalClickGui.hpp
    MOC
        src/MinimalClickGui.hpp
   DEPS_PKGCONFIG vizkit3d  maps-viz base-viz sbpl_spline_primitives-viz base-types vizkit3d_debug_drawings)
