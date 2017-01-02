find_package(PCL 1.7 REQUIRED COMPONENTS segmentation)

add_definitions(-DBOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS)

rock_library(ugv_nav4d
    SOURCES 
        DiscreteTheta.cpp
        EnvironmentXYZTheta.cpp
        Planner.cpp
        PreComputedMotions.cpp
        TraversabilityGenerator3d.cpp
        TravGenDebugData.cpp
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
    DEPS_PKGCONFIG 
        base-types 
        sbpl
        backward
        motion_planning_libraries
        dwa
        maps
        envire_core
        pcl_segmentation-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}
)
