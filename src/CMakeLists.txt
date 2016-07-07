rock_library(ugv_nav4d
    SOURCES 
        EnvironmentXYZTheta.cpp
    HEADERS 
        EnvironmentXYZTheta.hpp
    DEPS_PKGCONFIG 
        base-types 
        trav_gen_3d
        sbpl
        backward
        motion_planning_libraries
        dwa
)

rock_executable(ugv_nav4d_bin Main.cpp
   DEPS ugv_nav4d
   DEPS_PKGCONFIG vizkit3d ugv_nav4d-viz maps-viz)

