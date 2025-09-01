rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL)

include(RockQt)

find_package(PCL 1.7 REQUIRED COMPONENTS common io)
IF("${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}" VERSION_LESS 1.14)
    SET(PCL_VERSION_SUFFIX "-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}")
ENDIF()

if(ENABLE_DEBUG_DRAWINGS AND vizkit3d_debug_drawings-commands_FOUND)
	# These are (only) required for building the GUIs:
	list(APPEND DEPS_PKGCONFIG_QT vizkit3d_debug_drawings)
endif()

rock_qt_library(
    TARGETPREFIX ugv_nav4d_gui
    QT4_SUFFIX ""
    MISSINGQTDEPS_NOBUILD
    SOURCES
        PlannerGui.cpp
    HEADERS
        PlannerGui.h
    MOC
        PlannerGui.h
    DEPS ugv_nav4d
    DEPS_QT4 Qt4::QtCore
    DEPS_QT5 Qt5::Core
    DEPS_PKGCONFIG
                       base-types
                       pcl_common${PCL_VERSION_SUFFIX}
                       pcl_io${PCL_VERSION_SUFFIX}
    DEPS_PKGCONFIG_QT  vizkit3d
                       vizkit3d-viz
                       traversability_generator3d-viz
                       maps-viz
                       base-viz
                       sbpl_spline_primitives-viz
                       trajectory_follower-viz
                       ${DEPS_PKGCONFIG_QT}
)

rock_qt_executable(
    TARGETPREFIX ugv_nav4d_bin
    QT4_SUFFIX ""
    MISSINGQTDEPS_NOBUILD
    SOURCES
        Main.cpp
    DEPS ugv_nav4d
    DEPS_QT ugv_nav4d_gui
)

rock_qt_executable(
    TARGETPREFIX ugv_nav4d_replay
    QT4_SUFFIX ""
    MISSINGQTDEPS_NOBUILD
    SOURCES
        ReplayDump.cpp
    DEPS ugv_nav4d
    DEPS_QT ugv_nav4d_gui
    DEPS_PKGCONFIG
        base-types
    DEPS_PKGCONFIG_QT vizkit3d  maps-viz base-viz
        sbpl_spline_primitives-viz trajectory_follower-viz
)

if(TARGET ugv_nav4d_gui)
    if (ENABLE_DEBUG_DRAWINGS AND vizkit3d_debug_drawings-commands_FOUND AND vizkit3d_debug_drawings_FOUND)
        message(STATUS "Gui using Qt4: enabled, with debug drawings")
    else()
        message(STATUS "Gui using Qt4: enabled, without debug drawings")
    endif()
else(ROCK_QT_VERSION_4 AND vizkit3d_FOUND)
    message(STATUS "Gui using Qt4: disabled")
endif()

if(TARGET ugv_nav4d_gui-qt5)
    if (ENABLE_DEBUG_DRAWINGS AND vizkit3d_debug_drawings-commands_FOUND AND vizkit3d_debug_drawings-qt5_FOUND)
        message(STATUS "Gui using Qt5: enabled, with debug drawings")
    else()
        message(STATUS "Gui using Qt5: enabled, without debug drawings")
    endif()
else()
    message(STATUS "Gui using Qt5: disabled")
endif()
