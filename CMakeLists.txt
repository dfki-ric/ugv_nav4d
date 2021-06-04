# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
find_package(Rock)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
rock_init(ugv_nav4d 0.1)
add_definitions(-DENABLE_DEBUG_DRAWINGS)
rock_standard_layout()
