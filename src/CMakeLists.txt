find_package(PCL 1.7 REQUIRED COMPONENTS segmentation)

# add_definitions(-DBOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

rock_library(ugv_nav4d
    SOURCES 
        AreaExplorer.cpp
        CollisionCheck.cpp
        DiscreteTheta.cpp
        EnvironmentXYZTheta.cpp
        FrontierGenerator.cpp
        OrientedBox.cpp
        PathStatistics.cpp
        Planner.cpp
        PlannerDump.cpp
        PreComputedMotions.cpp
        TraversabilityGenerator3d.cpp
        Dijkstra.cpp
        ObstacleMapGenerator3D.cpp
    HEADERS 
        AreaExplorer.hpp
        Config.hpp
        CollisionCheck.hpp
        DiscreteTheta.hpp
        FrontierGenerator.hpp
        EnvironmentXYZTheta.hpp
        OrientedBox.hpp
        Planner.hpp
        PreComputedMotions.hpp
        TraversabilityGenerator3d.hpp
        TravGenNode.hpp
        TraversabilityConfig.hpp
        TravMapBfsVisitor.hpp
        Dijkstra.hpp
        ObstacleMapGenerator3D.hpp
    DEPS_PKGCONFIG 
        base-types 
        sbpl
        sbpl_spline_primitives
        maps
        pcl_segmentation-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}
        vizkit3d_debug_drawings
)

add_subdirectory(gui)
