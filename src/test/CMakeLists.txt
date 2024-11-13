find_package(Boost REQUIRED COMPONENTS system filesystem serialization)

add_executable(test_Planner test_Planner.cpp)
add_executable(test_EnvironmentXYZTheta test_EnvironmentXYZTheta.cpp)
add_executable(test_DiscreteTheta test_DiscreteTheta.cpp)
add_executable(test_PreComputedMotions test_PreComputedMotions.cpp)

set(DEPS Boost::filesystem)

if (ROCK_QT_VERSION_4)
	target_link_libraries(test_Planner PRIVATE
							ugv_nav4d ${DEPS})
	target_link_libraries(test_EnvironmentXYZTheta PRIVATE
							ugv_nav4d ${DEPS})
	target_link_libraries(test_DiscreteTheta PRIVATE
							ugv_nav4d)
	target_link_libraries(test_PreComputedMotions PRIVATE
							ugv_nav4d ${DEPS})
endif()

if (ROCK_QT_VERSION_5)
	target_link_libraries(test_Planner PRIVATE
						  	ugv_nav4d-qt5 ${DEPS})
	target_link_libraries(test_EnvironmentXYZTheta PRIVATE
	                        ugv_nav4d-qt5 ${DEPS})
	target_link_libraries(test_DiscreteTheta PRIVATE
							ugv_nav4d-qt5)
	target_link_libraries(test_PreComputedMotions PRIVATE
							ugv_nav4d-qt5 ${DEPS})							
endif()

# Install the binaries
install(TARGETS test_Planner EXPORT test_Planner-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

install(TARGETS test_EnvironmentXYZTheta EXPORT test_EnvironmentXYZTheta-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

install(TARGETS test_DiscreteTheta EXPORT test_DiscreteTheta-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

install(TARGETS test_PreComputedMotions EXPORT test_PreComputedMotions-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)