
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")

rock_executable(ugv_nav4d_bin
    SOURCES
        Main.cpp
        PlannerGui.cpp
    HEADERS
        PlannerGui.h
    MOC
        PlannerGui.h        
   DEPS ugv_nav4d 
   DEPS_PKGCONFIG vizkit3d  maps-viz base-viz sbpl_spline_primitives-viz base-types)

rock_executable(ugv_nav4d_replay
    SOURCES
        ReplayDump.cpp
        PlannerGui.cpp
    HEADERS
        PlannerGui.h
    MOC
        PlannerGui.h        
   DEPS ugv_nav4d 
   DEPS_PKGCONFIG vizkit3d  maps-viz base-viz sbpl_spline_primitives-viz base-types)

rock_executable(frontier_test_bin
    SOURCES
        FrontierTest.cpp
        FrontierTestGui.cpp
    HEADERS
        FrontierTestGui.hpp
    MOC
        FrontierTestGui.hpp      
   DEPS ugv_nav4d 
   DEPS_PKGCONFIG vizkit3d  maps-viz base-viz sbpl_spline_primitives-viz base-types)
   
