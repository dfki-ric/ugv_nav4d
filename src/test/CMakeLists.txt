find_package(GTest REQUIRED)
add_executable(test_ugv_nav4d test_ugv_nav4d.cpp)

if (ROCK_QT_VERSION_4)
	target_link_libraries(test_ugv_nav4d GTest::GTest ugv_nav4d)
endif()

if (ROCK_QT_VERSION_5)
	target_link_libraries(test_ugv_nav4d GTest::GTest ugv_nav4d-qt5)
endif()

# Install the binaries
install(TARGETS test_ugv_nav4d EXPORT test_ugv_nav4d-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)
