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


rock_executable(test_bin test.cpp
   DEPS ugv_nav4d)

