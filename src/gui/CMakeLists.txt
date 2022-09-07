set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")

rock_library(ugv_nav4d_gui
    SOURCES
        PlannerGui.cpp
    HEADERS
        PlannerGui.h
    MOC
        PlannerGui.h        
   DEPS ugv_nav4d 
   DEPS_PKGCONFIG vizkit3d  
                  vizkit3d-viz  
                  maps-viz 
                  base-viz 
                  sbpl_spline_primitives-viz 
                  base-types 
                  trajectory_follower-viz 
)

rock_executable(ugv_nav4d_bin
    SOURCES
        Main.cpp
   DEPS ugv_nav4d ugv_nav4d_gui
)

rock_executable(ugv_nav4d_replay
    SOURCES
        ReplayDump.cpp
   DEPS ugv_nav4d ugv_nav4d_gui
)
