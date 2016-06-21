rock_library(ugv_nav4d
   SOURCES EnvironmentXYZTheta.cpp
   HEADERS EnvironmentXYZTheta.hpp
    DEPS_PKGCONFIG 
        base-types 
        trav_gen_3d
        sbpl
        )

#rock_executable(ugv_nav4d_bin Main.cpp
#    DEPS ugv_nav4d)

