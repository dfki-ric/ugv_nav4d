find_package(OpenMP REQUIRED)

set(DEPS_PKGCONFIG_LIST
    base-types
    sbpl
    sbpl_spline_primitives
    traversability_generator3d
    trajectory_follower
    collide
)

if(ENABLE_DEBUG_DRAWINGS AND vizkit3d_debug_drawings-commands_FOUND)
    list(APPEND DEPS_PKGCONFIG_LIST vizkit3d_debug_drawings-commands)
endif()

set(ADDITIONAL_LIBS "")
if(OpenMP_CXX_FOUND)
	message(STATUS "OpenMP is found: ${OpenMP_CXX_FOUND}")
    list(APPEND ADDITIONAL_LIBS OpenMP::OpenMP_CXX)
endif()

rock_library(ugv_nav4d
    SOURCES 
        DiscreteTheta.cpp
        EnvironmentXYZTheta.cpp
        PathStatistic.cpp
        Planner.cpp
        PlannerDump.cpp
        PreComputedMotions.cpp
        Dijkstra.cpp
        DebugDrawingDeclarations.cpp
    HEADERS 
        Mobility.hpp
        DiscreteTheta.hpp
        EnvironmentXYZTheta.hpp
        PathStatistic.hpp
        Planner.hpp
        PlannerConfig.hpp
        PreComputedMotions.hpp
        Dijkstra.hpp
    DEPS_PKGCONFIG 
        ${DEPS_PKGCONFIG_LIST}
    LIBS 
        ${ADDITIONAL_LIBS}
)

add_subdirectory(gui)

if(TESTS_ENABLED)
    message(STATUS "TESTS_ENABLED is defined with value: ${TESTS_ENABLED}")
    add_subdirectory(test)
else()
    message(STATUS "TESTS_ENABLED is set to OFF. Skipped!")
endif()
