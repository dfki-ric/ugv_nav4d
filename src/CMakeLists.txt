rock_library(ugv_nav4d
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(ugv_nav4d_bin Main.cpp
    DEPS ugv_nav4d)

