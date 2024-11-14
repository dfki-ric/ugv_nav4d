add_executable(test_ugv_nav4d test_ugv_nav4d.cpp)
add_executable(test_EnvironmentXYZTheta test_EnvironmentXYZTheta.cpp)

target_include_directories(test_ugv_nav4d PUBLIC ${Boost_INCLUDE_DIR})
target_include_directories(test_EnvironmentXYZTheta PUBLIC ${Boost_INCLUDE_DIR})


if (ROCK_QT_VERSION_4)
	target_link_libraries(test_ugv_nav4d PRIVATE
							ugv_nav4d)
	target_link_libraries(test_EnvironmentXYZTheta PRIVATE
							ugv_nav4d)							
endif()

if (ROCK_QT_VERSION_5)
	target_link_libraries(test_ugv_nav4d PRIVATE
						  ugv_nav4d-qt5)
	target_link_libraries(test_EnvironmentXYZTheta PRIVATE
						  ugv_nav4d-qt5)		
endif()

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
