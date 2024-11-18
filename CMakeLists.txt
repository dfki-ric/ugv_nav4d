# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.

cmake_minimum_required(VERSION 3.9)
project(ugv_nav4d
        VERSION 1.0
        DESCRIPTION "A 4D (X,Y,Z, Theta) Planner for unmaned ground vehicles (UGVs).")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Rock)
rock_init()
rock_feature(NOCURDIR)

pkg_check_modules(vizkit3d_debug_drawings-commands vizkit3d_debug_drawings-commands)

if(ENABLE_DEBUG_DRAWINGS)
    message(STATUS "ENABLE_DEBUG_DRAWINGS is defined with value: ${ENABLE_DEBUG_DRAWINGS}")
    if(vizkit3d_debug_drawings-commands_FOUND)
        add_definitions(-DENABLE_DEBUG_DRAWINGS)
        message(STATUS "Building with vizkit3d_debug_drawings support in library")
    else()
        message(STATUS "Cannot find vizkit3d_debug_drawings, ignoring ENABLE_DEBUG_DRAWINGS")
    endif()
else()
    message(STATUS "ENABLE_DEBUG_DRAWINGS is set to OFF. Skipped!")
endif()

if(INSTALL_DEPS)

  execute_process(COMMAND bash install_os_dependencies.bash
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/source_dependencies)
  execute_process(COMMAND bash build.bash ${CMAKE_INSTALL_PREFIX}
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/source_dependencies)
  set(ENV{PKG_CONFIG_PATH} ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/)
  set(ENV{CMAKE_PREFIX_PATH} ${CMAKE_INSTALL_PREFIX}/share/rock/cmake/)
endif()

rock_standard_layout()
