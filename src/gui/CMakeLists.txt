rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL)

pkg_check_modules(vizkit3d vizkit3d)
pkg_check_modules(vizkit3d-qt5 vizkit3d-qt5)

pkg_check_modules(vizkit3d_debug_drawings vizkit3d_debug_drawings)
pkg_check_modules(vizkit3d_debug_drawings-qt5 vizkit3d_debug_drawings-qt5)

pkg_check_modules(maps-viz maps-viz)
pkg_check_modules(maps-viz-qt5 maps-viz-qt5)

pkg_check_modules(base-viz base-viz)
pkg_check_modules(base-viz-qt5 base-viz-qt5)

find_package(PCL 1.7 REQUIRED COMPONENTS common io)
IF("${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}" VERSION_LESS 1.14)
    SET(PCL_VERSION_SUFFIX "-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}")
ENDIF()

if(ENABLE_DEBUG_DRAWINGS AND vizkit3d_debug_drawings-commands_FOUND)
	# These are (only) required for building the GUIs:
	if(vizkit3d_debug_drawings_FOUND)
		list(APPEND DEPS_PKGCONFIG_QT4 vizkit3d_debug_drawings)
	endif()
	if(vizkit3d_debug_drawings-qt5_FOUND)
		list(APPEND DEPS_PKGCONFIG_QT5 vizkit3d_debug_drawings-qt5)
	endif()
endif()

if(ROCK_QT_VERSION_4 AND vizkit3d_FOUND AND maps-viz_FOUND AND base-viz_FOUND)
    rock_library(ugv_nav4d_gui
        SOURCES
            PlannerGui.cpp
        HEADERS
            PlannerGui.h
        MOC
            PlannerGui.h
        DEPS ugv_nav4d Qt4::QtCore
        DEPS_PKGCONFIG vizkit3d
                       vizkit3d-viz
                       traversability_generator3d-viz
                       maps-viz
                       base-viz
                       sbpl_spline_primitives-viz
                       base-types
                       trajectory_follower-viz
                       pcl_common${PCL_VERSION_SUFFIX}
                       pcl_io${PCL_VERSION_SUFFIX}
                       ${DEPS_PKGCONFIG_QT4}
    )

    rock_executable(ugv_nav4d_bin
        SOURCES
                Main.cpp
        DEPS ugv_nav4d ugv_nav4d_gui
    )

    rock_executable(ugv_nav4d_replay
        SOURCES
                ReplayDump.cpp
        DEPS ugv_nav4d ugv_nav4d_gui
        DEPS_PKGCONFIG vizkit3d  maps-viz base-viz
        sbpl_spline_primitives-viz base-types trajectory_follower-viz
    )
    if (ENABLE_DEBUG_DRAWINGS AND vizkit3d_debug_drawings-commands_FOUND AND vizkit3d_debug_drawings_FOUND)
        message(STATUS "Gui using Qt4: enabled, with debug drawings")
    else()
        message(STATUS "Gui using Qt4: enabled, without debug drawings")
    endif()
else(ROCK_QT_VERSION_4 AND vizkit3d_FOUND)
    message(STATUS "Gui using Qt4: disabled")
endif(ROCK_QT_VERSION_4 AND vizkit3d_FOUND)

if(ROCK_QT_VERSION_5 AND vizkit3d-qt5_FOUND AND maps-viz-qt5_FOUND AND base-viz-qt5_FOUND)
    rock_library(ugv_nav4d_gui-qt5
        SOURCES
            PlannerGui.cpp
        HEADERS
            PlannerGui.h
        MOC5
            PlannerGui.h
        DEPS ugv_nav4d Qt5::Core
        DEPS_PKGCONFIG vizkit3d-qt5
                       vizkit3d-viz-qt5
                       traversability_generator3d-viz-qt5
                       maps-viz-qt5
                       base-viz-qt5
                       sbpl_spline_primitives-viz-qt5
                       base-types
                       trajectory_follower-viz-qt5
                       pcl_common${PCL_VERSION_SUFFIX}
                       pcl_io${PCL_VERSION_SUFFIX}
                       ${DEPS_PKGCONFIG_QT5}
    )

    rock_executable(ugv_nav4d_bin-qt5
        SOURCES
            Main.cpp
        DEPS ugv_nav4d ugv_nav4d_gui-qt5
    )

    rock_executable(ugv_nav4d_replay-qt5
        SOURCES
            ReplayDump.cpp
        DEPS ugv_nav4d ugv_nav4d_gui-qt5
        DEPS_PKGCONFIG vizkit3d-qt5  maps-viz-qt5 base-viz-qt5
        sbpl_spline_primitives-viz-qt5 base-types trajectory_follower-viz-qt5
    )
    if (ENABLE_DEBUG_DRAWINGS AND vizkit3d_debug_drawings-commands_FOUND AND vizkit3d_debug_drawings-qt5_FOUND)
        message(STATUS "Gui using Qt5: enabled, with debug drawings")
    else()
        message(STATUS "Gui using Qt5: enabled, without debug drawings")
    endif()
else()
    message(STATUS "Gui using Qt5: disabled")
endif(ROCK_QT_VERSION_5 AND vizkit3d-qt5_FOUND)
