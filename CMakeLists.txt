# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.

cmake_minimum_required(VERSION 3.9)
project(ugv_nav4d
        VERSION 1.0
        DESCRIPTION "GF")
find_package(Rock)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(ENABLE_DEBUG_DRAWINGS)
  add_definitions(-DENABLE_DEBUG_DRAWINGS)
endif()

rock_init()
rock_feature(NOCURDIR)
add_definitions(-DENABLE_DEBUG_DRAWINGS)
rock_standard_layout()
