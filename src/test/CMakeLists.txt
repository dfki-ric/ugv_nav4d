find_package(Boost REQUIRED COMPONENTS filesystem serialization)

rock_executable(test_Planner
    SOURCES test_Planner.cpp
    DEPS_PKGCONFIG ugv_nav4d
)

rock_executable(test_EnvironmentXYZTheta
    SOURCES test_EnvironmentXYZTheta.cpp
	DEPS Boost::filesystem Boost::serialization
    DEPS_PKGCONFIG ugv_nav4d
)

rock_executable(test_DiscreteTheta
    SOURCES test_DiscreteTheta.cpp
    DEPS_PKGCONFIG ugv_nav4d
)

rock_executable(test_PreComputedMotions
    SOURCES test_PreComputedMotions.cpp
    DEPS_PKGCONFIG ugv_nav4d
)
