find_package(Boost REQUIRED COMPONENTS filesystem)


rock_executable(test_ugv_nav4d
    SOURCES test_ugv_nav4d.cpp
    DEPS_PKGCONFIG ugv_nav4d
)

rock_executable(test_EnvironmentXYZTheta
    SOURCES test_EnvironmentXYZTheta.cpp
    DEPS Boost::filesystem
    DEPS_PKGCONFIG ugv_nav4d
)
