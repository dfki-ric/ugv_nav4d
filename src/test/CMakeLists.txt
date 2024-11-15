find_package(Boost REQUIRED COMPONENTS filesystem)

add_executable(test_ugv_nav4d test_ugv_nav4d.cpp)
add_executable(test_EnvironmentXYZTheta test_EnvironmentXYZTheta.cpp)


target_link_libraries(test_ugv_nav4d           PRIVATE ugv_nav4d Boost::filesystem)
target_link_libraries(test_EnvironmentXYZTheta PRIVATE ugv_nav4d Boost::filesystem)


# Install the binaries
install(TARGETS test_ugv_nav4d EXPORT test_ugv_nav4d-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

install(TARGETS test_EnvironmentXYZTheta EXPORT test_EnvironmentXYZTheta-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

