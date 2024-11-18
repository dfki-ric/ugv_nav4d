rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL)

find_package(PCL 1.7 REQUIRED COMPONENTS common io)
IF("${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}" VERSION_LESS 1.14)
    SET(PCL_VERSION_SUFFIX "-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}")
ENDIF()

if(ROCK_QT_VERSION_4)
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
endif(ROCK_QT_VERSION_4)

if(ROCK_QT_VERSION_5)
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
endif(ROCK_QT_VERSION_5)
