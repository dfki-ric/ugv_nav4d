find_package(GTest REQUIRED)
add_executable(test_ugv_nav4d test_ugv_nav4d.cpp)
target_link_libraries(test_ugv_nav4d GTest::GTest ugv_nav4d)

# Install the binaries
install(TARGETS test_ugv_nav4d EXPORT test_ugv_nav4d-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)
