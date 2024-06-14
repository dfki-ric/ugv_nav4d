rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

if (ROCK_QT_VERSION_4)
	rock_library(ugv_nav4d
	    SOURCES
		DiscreteTheta.cpp
		EnvironmentXYZTheta.cpp
		PathStatistic.cpp
		Planner.cpp
		PlannerDump.cpp
		PreComputedMotions.cpp
		Dijkstra.cpp
		ObstacleMapGenerator3D.cpp
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
		ObstacleMapGenerator3D.hpp
	    DEPS_PKGCONFIG
		base-types
		sbpl
		sbpl_spline_primitives
		traversability_generator3d
		vizkit3d_debug_drawings
		trajectory_follower
	)
endif()

if (ROCK_QT_VERSION_5)
	rock_library(ugv_nav4d-qt5
	    SOURCES 
		DiscreteTheta.cpp
		EnvironmentXYZTheta.cpp
		PathStatistic.cpp
		Planner.cpp
		PlannerDump.cpp
		PreComputedMotions.cpp
		Dijkstra.cpp
		ObstacleMapGenerator3D.cpp
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
		ObstacleMapGenerator3D.hpp
	    DEPS_PKGCONFIG 
		base-types 
		sbpl
		sbpl_spline_primitives
		traversability_generator3d
		vizkit3d_debug_drawings-qt5
		trajectory_follower
	)
endif()

add_subdirectory(gui)
#add_subdirectory(test)
