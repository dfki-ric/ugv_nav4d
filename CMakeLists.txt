# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
find_package(Rock)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
rock_init(ugv_nav4d 0.1)
rock_standard_layout()

rock_executable(ugv_nav4d_bin
    SOURCES
        src/Main.cpp
        src/PlannerGui.cpp
    HEADERS
        src/PlannerGui.h
    MOC
        src/PlannerGui.h        
   DEPS ugv_nav4d ugv_nav4d-viz
   DEPS_PKGCONFIG vizkit3d  maps-viz base-viz motion_planning_libraries-viz base-types)
