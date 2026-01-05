find_package(Boost REQUIRED COMPONENTS filesystem serialization)

add_executable(test_Planner test_Planner.cpp)
target_link_libraries(test_Planner
  ugv_nav4d
  Boost::filesystem 
)

add_executable(test_EnvironmentXYZTheta test_EnvironmentXYZTheta.cpp)
target_link_libraries(test_EnvironmentXYZTheta
  ugv_nav4d
  Boost::filesystem 
  Boost::serialization
)

add_executable(test_DiscreteTheta test_DiscreteTheta.cpp)
target_link_libraries(test_DiscreteTheta
  ugv_nav4d
)

add_executable(test_PreComputedMotions test_PreComputedMotions.cpp)
target_link_libraries(test_PreComputedMotions
  ugv_nav4d
  Boost::filesystem
)