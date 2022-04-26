rock_find_qt5(Core Gui Widgets REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fPIC")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")


rock_library(ugv_nav4d_gui
    SOURCES
        PlannerGui.cpp
    HEADERS
        PlannerGui.h
    MOC
        PlannerGui.h        
   DEPS ugv_nav4d 
   DEPS_PKGCONFIG vizkit3d  maps-viz base-viz sbpl_spline_primitives-viz base-types trajectory_follower-viz
)

rock_executable(ugv_nav4d_bin
    SOURCES
        Main.cpp
   DEPS ugv_nav4d ugv_nav4d_gui
   DEPS_PKGCONFIG vizkit3d  maps-viz base-viz sbpl_spline_primitives-viz base-types trajectory_follower-viz)

rock_executable(ugv_nav4d_replay
    SOURCES
        ReplayDump.cpp
   DEPS ugv_nav4d ugv_nav4d_gui
   DEPS_PKGCONFIG vizkit3d  maps-viz base-viz sbpl_spline_primitives-viz base-types trajectory_follower-viz)