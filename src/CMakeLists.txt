find_package(PCL 1.7 REQUIRED COMPONENTS segmentation)

rock_library(ugv_nav4d
    SOURCES 
        DiscreteTheta.cpp
        EnvironmentXYZTheta.cpp
        Planner.cpp
        PreComputedMotions.cpp
        TraversabilityGenerator3d.cpp
    HEADERS 
        DiscreteTheta.hpp
        EnvironmentXYZTheta.hpp
        Planner.hpp
        PreComputedMotions.hpp
        TraversabilityGenerator3d.hpp
        TraversabilityConfig.hpp
    DEPS_PKGCONFIG 
        base-types 
        sbpl
        backward
        motion_planning_libraries
        dwa
        maps
        pcl_segmentation-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}
)
