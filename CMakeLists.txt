# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.0)
find_package(Rock)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(ugv_nav4d VERSION 1.0 DESCRIPTION "A library for planning trajectories in X,Y, and Theta")
rock_init()
add_definitions(-DENABLE_DEBUG_DRAWINGS)
rock_standard_layout()
