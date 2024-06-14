find_package(PCL 1.7 REQUIRED COMPONENTS common io)

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
                      pcl_common-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}
                      pcl_io-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}
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
       DEPS ugv_nav4d-qt5 Qt5::Core
       DEPS_PKGCONFIG vizkit3d-qt5
                      vizkit3d-viz-qt5
                      maps-viz-qt5
                      base-viz-qt5
                      sbpl_spline_primitives-viz-qt5
                      base-types
                      trajectory_follower-viz-qt5
                      pcl_common-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}
                      pcl_io-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}
    )
endif(ROCK_QT_VERSION_5)

if(ROCK_QT_VERSION_4)
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
    if(ROCK_QT_VERSION_5)
        rock_executable(ugv_nav4d_bin-qt5
            SOURCES
                Main.cpp
           DEPS ugv_nav4d-qt5 ugv_nav4d_gui-qt5
        )

        rock_executable(ugv_nav4d_replay-qt5
            SOURCES
                ReplayDump.cpp
           DEPS ugv_nav4d-qt5 ugv_nav4d_gui-qt5
           DEPS_PKGCONFIG vizkit3d-qt5  maps-viz-qt5 base-viz-qt5
           sbpl_spline_primitives-viz-qt5 base-types trajectory_follower-viz-qt5
        )
    endif(ROCK_QT_VERSION_5)
elseif(ROCK_QT_VERSION_5)
    rock_executable(ugv_nav4d_bin
        SOURCES
            Main.cpp
       DEPS ugv_nav4d-qt5 ugv_nav4d_gui-qt5
    )

    rock_executable(ugv_nav4d_replay
        SOURCES
            ReplayDump.cpp
       DEPS ugv_nav4d-qt5 ugv_nav4d_gui-qt5
       DEPS_PKGCONFIG vizkit3d-qt5  maps-viz-qt5 base-viz-qt5
       sbpl_spline_primitives-viz-qt5 base-types trajectory_follower-viz-qt5
    )
endif()
