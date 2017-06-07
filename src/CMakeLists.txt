find_package(PCL 1.7 REQUIRED COMPONENTS segmentation)

# add_definitions(-DBOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS)
add_definitions(-DENABLE_DEBUG_DRAWINGS)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


rock_library(ugv_nav4d
    SOURCES 
        DiscreteTheta.cpp
        EnvironmentXYZTheta.cpp
        Planner.cpp
        PreComputedMotions.cpp
        TraversabilityGenerator3d.cpp
        TravGenDebugData.cpp
        FrontierGenerator.cpp
        CollisionCheck.cpp
    HEADERS 
        DiscreteTheta.hpp
        EnvironmentXYZTheta.hpp
        Planner.hpp
        PreComputedMotions.hpp
        TraversabilityGenerator3d.hpp
        TravGenNode.hpp
        TraversabilityConfig.hpp
        TravGenDebugData.hpp
        UgvDebug.hpp
        EnvironmentXYZThetaDebugData.hpp
        TravMapBfsVisitor.hpp
        FrontierGenerator.hpp
        CollisionCheck.hpp
    DEPS_PKGCONFIG 
        base-types 
        sbpl
        backward
        motion_planning_libraries
        dwa
        maps
        envire_core
        pcl_segmentation-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}
        vizkit3d_debug_drawings
)
