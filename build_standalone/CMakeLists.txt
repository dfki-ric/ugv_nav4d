cmake_minimum_required(VERSION 3.10)
include(ExternalProject)

ExternalProject_Add(planning-ugv_nav4d
    PREFIX       planning/ugv_nav4d
    STAMP_DIR    ${CMAKE_CURRENT_BINARY_DIR}/planning/ugv_nav4d/stamp
    DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/planning/ugv_nav4d/src
    SOURCE_DIR   ${CMAKE_CURRENT_BINARY_DIR}/planning/ugv_nav4d/src
    BINARY_DIR   ${CMAKE_CURRENT_BINARY_DIR}/planning/ugv_nav4d/build
    INSTALL_DIR         ${CMAKE_INSTALL_PREFIX}
    GIT_REPOSITORY      https://git.hb.dfki.de/dfki-planning/ugv_nav4d.git
    GIT_TAG             feature/qt4-qt5
    CMAKE_CACHE_ARGS    -DENV{PKG_CONFIG_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/
                        -DENV{CMAKE_PREFIX_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/share/rock/cmake/
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>  -DCMAKE_BUILD_TYPE=RELEASE 
)
add_dependencies(planning-ugv_nav4d base-cmake planning-sbpl planning-sbpl_spline_primitives planning-traversability_generator3d gui-vizkit3d_debug_drawings control-trajectory_follower external-googletest )

ExternalProject_Add(base-cmake
    PREFIX       base/cmake
    STAMP_DIR    ${CMAKE_CURRENT_BINARY_DIR}/base/cmake/stamp
    DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/base/cmake/src
    SOURCE_DIR   ${CMAKE_CURRENT_BINARY_DIR}/base/cmake/src
    BINARY_DIR   ${CMAKE_CURRENT_BINARY_DIR}/base/cmake/build
    INSTALL_DIR         ${CMAKE_INSTALL_PREFIX}
    GIT_REPOSITORY      https://github.com/rock-core/base-cmake.git
    GIT_TAG             master
    CMAKE_CACHE_ARGS    -DENV{PKG_CONFIG_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/
                        -DENV{CMAKE_PREFIX_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/share/rock/cmake/
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>  -DCMAKE_BUILD_TYPE=RELEASE 
)

ExternalProject_Add(planning-sbpl
    PREFIX       planning/sbpl
    STAMP_DIR    ${CMAKE_CURRENT_BINARY_DIR}/planning/sbpl/stamp
    DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/planning/sbpl/src
    SOURCE_DIR   ${CMAKE_CURRENT_BINARY_DIR}/planning/sbpl/src
    BINARY_DIR   ${CMAKE_CURRENT_BINARY_DIR}/planning/sbpl/build
    INSTALL_DIR         ${CMAKE_INSTALL_PREFIX}
    GIT_REPOSITORY      https://github.com/sbpl/sbpl.git
    GIT_TAG             master
    CMAKE_CACHE_ARGS    -DENV{PKG_CONFIG_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/
                        -DENV{CMAKE_PREFIX_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/share/rock/cmake/
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>  -DCMAKE_BUILD_TYPE=RELEASE 
)

ExternalProject_Add(planning-sbpl_spline_primitives
    PREFIX       planning/sbpl_spline_primitives
    STAMP_DIR    ${CMAKE_CURRENT_BINARY_DIR}/planning/sbpl_spline_primitives/stamp
    DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/planning/sbpl_spline_primitives/src
    SOURCE_DIR   ${CMAKE_CURRENT_BINARY_DIR}/planning/sbpl_spline_primitives/src
    BINARY_DIR   ${CMAKE_CURRENT_BINARY_DIR}/planning/sbpl_spline_primitives/build
    INSTALL_DIR         ${CMAKE_INSTALL_PREFIX}
    GIT_REPOSITORY      https://github.com/pierrewillenbrockdfki/planning-sbpl_spline_primitives.git
    GIT_TAG             feature/qt4-qt5
    CMAKE_CACHE_ARGS    -DENV{PKG_CONFIG_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/
                        -DENV{CMAKE_PREFIX_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/share/rock/cmake/
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>  -DCMAKE_BUILD_TYPE=RELEASE 
)
add_dependencies(planning-sbpl_spline_primitives base-cmake base-types )

ExternalProject_Add(planning-traversability_generator3d
    PREFIX       planning/traversability_generator3d
    STAMP_DIR    ${CMAKE_CURRENT_BINARY_DIR}/planning/traversability_generator3d/stamp
    DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/planning/traversability_generator3d/src
    SOURCE_DIR   ${CMAKE_CURRENT_BINARY_DIR}/planning/traversability_generator3d/src
    BINARY_DIR   ${CMAKE_CURRENT_BINARY_DIR}/planning/traversability_generator3d/build
    INSTALL_DIR         ${CMAKE_INSTALL_PREFIX}
    GIT_REPOSITORY      https://git.hb.dfki.de/dfki-planning/traversability_generator3d.git
    GIT_TAG             master
    CMAKE_CACHE_ARGS    -DENV{PKG_CONFIG_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/
                        -DENV{CMAKE_PREFIX_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/share/rock/cmake/
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>  -DCMAKE_BUILD_TYPE=RELEASE 
)
add_dependencies(planning-traversability_generator3d base-cmake slam-maps gui-vizkit3d_debug_drawings )

ExternalProject_Add(gui-vizkit3d_debug_drawings
    PREFIX       gui/vizkit3d_debug_drawings
    STAMP_DIR    ${CMAKE_CURRENT_BINARY_DIR}/gui/vizkit3d_debug_drawings/stamp
    DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/gui/vizkit3d_debug_drawings/src
    SOURCE_DIR   ${CMAKE_CURRENT_BINARY_DIR}/gui/vizkit3d_debug_drawings/src
    BINARY_DIR   ${CMAKE_CURRENT_BINARY_DIR}/gui/vizkit3d_debug_drawings/build
    INSTALL_DIR         ${CMAKE_INSTALL_PREFIX}
    GIT_REPOSITORY      https://github.com/pierrewillenbrockdfki/gui-vizkit3d_debug_drawings.git
    GIT_TAG             feature/qt4-qt5
    CMAKE_CACHE_ARGS    -DENV{PKG_CONFIG_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/
                        -DENV{CMAKE_PREFIX_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/share/rock/cmake/
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>  -DWITH_PORTS=OFF -DCMAKE_BUILD_TYPE=RELEASE 
)
add_dependencies(gui-vizkit3d_debug_drawings base-cmake gui-vizkit3d gui-osgviz )

ExternalProject_Add(control-trajectory_follower
    PREFIX       control/trajectory_follower
    STAMP_DIR    ${CMAKE_CURRENT_BINARY_DIR}/control/trajectory_follower/stamp
    DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/control/trajectory_follower/src
    SOURCE_DIR   ${CMAKE_CURRENT_BINARY_DIR}/control/trajectory_follower/src
    BINARY_DIR   ${CMAKE_CURRENT_BINARY_DIR}/control/trajectory_follower/build
    INSTALL_DIR         ${CMAKE_INSTALL_PREFIX}
    GIT_REPOSITORY      https://github.com/pierrewillenbrockdfki/control-trajectory_follower.git
    GIT_TAG             limit_speed_at_end_of_traj-qt4-qt5
    CMAKE_CACHE_ARGS    -DENV{PKG_CONFIG_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/
                        -DENV{CMAKE_PREFIX_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/share/rock/cmake/
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>  -DCMAKE_BUILD_TYPE=RELEASE 
)
add_dependencies(control-trajectory_follower base-types external-sisl gui-osgviz )

ExternalProject_Add(external-googletest
    PREFIX       external/googletest
    STAMP_DIR    ${CMAKE_CURRENT_BINARY_DIR}/external/googletest/stamp
    DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/external/googletest/src
    SOURCE_DIR   ${CMAKE_CURRENT_BINARY_DIR}/external/googletest/src
    BINARY_DIR   ${CMAKE_CURRENT_BINARY_DIR}/external/googletest/build
    INSTALL_DIR         ${CMAKE_INSTALL_PREFIX}
    GIT_REPOSITORY      https://github.com/google/googletest.git
    GIT_TAG             main
    CMAKE_CACHE_ARGS    -DENV{PKG_CONFIG_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/
                        -DENV{CMAKE_PREFIX_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/share/rock/cmake/
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>  -DCMAKE_BUILD_TYPE=RELEASE 
)

ExternalProject_Add(base-types
    PREFIX       base/types
    STAMP_DIR    ${CMAKE_CURRENT_BINARY_DIR}/base/types/stamp
    DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/base/types/src
    SOURCE_DIR   ${CMAKE_CURRENT_BINARY_DIR}/base/types/src
    BINARY_DIR   ${CMAKE_CURRENT_BINARY_DIR}/base/types/build
    INSTALL_DIR         ${CMAKE_INSTALL_PREFIX}
    GIT_REPOSITORY      https://github.com/pierrewillenbrockdfki/base-types.git
    GIT_TAG             feature/qt4-qt5
    CMAKE_CACHE_ARGS    -DENV{PKG_CONFIG_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/
                        -DENV{CMAKE_PREFIX_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/share/rock/cmake/
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>  -DUSE_SISL=ON -DBINDINGS_RUBY=OFF -DROCK_VIZ_ENABLED=TRUE -DCMAKE_BUILD_TYPE=RELEASE 
)
add_dependencies(base-types base-cmake base-logging external-sisl gui-vizkit3d )

ExternalProject_Add(slam-maps
    PREFIX       slam/maps
    STAMP_DIR    ${CMAKE_CURRENT_BINARY_DIR}/slam/maps/stamp
    DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/slam/maps/src
    SOURCE_DIR   ${CMAKE_CURRENT_BINARY_DIR}/slam/maps/src
    BINARY_DIR   ${CMAKE_CURRENT_BINARY_DIR}/slam/maps/build
    INSTALL_DIR         ${CMAKE_INSTALL_PREFIX}
    GIT_REPOSITORY      https://github.com/haider8645/slam-maps.git
    GIT_TAG             feature/qt4-qt5
    CMAKE_CACHE_ARGS    -DENV{PKG_CONFIG_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/
                        -DENV{CMAKE_PREFIX_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/share/rock/cmake/
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>  -DCMAKE_BUILD_TYPE=RELEASE 
)
add_dependencies(slam-maps base-cmake base-types base-numeric base-boost_serialization gui-vizkit3d gui-osgviz )

ExternalProject_Add(gui-vizkit3d
    PREFIX       gui/vizkit3d
    STAMP_DIR    ${CMAKE_CURRENT_BINARY_DIR}/gui/vizkit3d/stamp
    DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/gui/vizkit3d/src
    SOURCE_DIR   ${CMAKE_CURRENT_BINARY_DIR}/gui/vizkit3d/src
    BINARY_DIR   ${CMAKE_CURRENT_BINARY_DIR}/gui/vizkit3d/build
    INSTALL_DIR         ${CMAKE_INSTALL_PREFIX}
    GIT_REPOSITORY      https://github.com/pierrewillenbrockdfki/gui-vizkit3d.git
    GIT_TAG             feature/qt4-qt5
    CMAKE_CACHE_ARGS    -DENV{PKG_CONFIG_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/
                        -DENV{CMAKE_PREFIX_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/share/rock/cmake/
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>  -DCMAKE_BUILD_TYPE=RELEASE 
)
add_dependencies(gui-vizkit3d base-cmake gui-osgviz gui-osg_qt5 gui-qtpropertybrowser )

ExternalProject_Add(gui-osgviz
    PREFIX       gui/osgviz
    STAMP_DIR    ${CMAKE_CURRENT_BINARY_DIR}/gui/osgviz/stamp
    DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/gui/osgviz/src
    SOURCE_DIR   ${CMAKE_CURRENT_BINARY_DIR}/gui/osgviz/src
    BINARY_DIR   ${CMAKE_CURRENT_BINARY_DIR}/gui/osgviz/build
    INSTALL_DIR         ${CMAKE_INSTALL_PREFIX}
    GIT_REPOSITORY      https://github.com/rock-core/gui-osgviz.git
    GIT_TAG             master
    CMAKE_CACHE_ARGS    -DENV{PKG_CONFIG_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/
                        -DENV{CMAKE_PREFIX_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/share/rock/cmake/
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>  -DCMAKE_BUILD_TYPE=RELEASE 
)
add_dependencies(gui-osgviz base-cmake )

ExternalProject_Add(external-sisl
    PREFIX       external/sisl
    STAMP_DIR    ${CMAKE_CURRENT_BINARY_DIR}/external/sisl/stamp
    DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/external/sisl/src
    SOURCE_DIR   ${CMAKE_CURRENT_BINARY_DIR}/external/sisl/src
    BINARY_DIR   ${CMAKE_CURRENT_BINARY_DIR}/external/sisl/build
    INSTALL_DIR         ${CMAKE_INSTALL_PREFIX}
    GIT_REPOSITORY      https://github.com/SINTEF-Geometry/SISL.git
    GIT_TAG             master
    CMAKE_CACHE_ARGS    -DENV{PKG_CONFIG_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/
                        -DENV{CMAKE_PREFIX_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/share/rock/cmake/
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>  -DCMAKE_BUILD_TYPE=RELEASE 
)

ExternalProject_Add(base-logging
    PREFIX       base/logging
    STAMP_DIR    ${CMAKE_CURRENT_BINARY_DIR}/base/logging/stamp
    DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/base/logging/src
    SOURCE_DIR   ${CMAKE_CURRENT_BINARY_DIR}/base/logging/src
    BINARY_DIR   ${CMAKE_CURRENT_BINARY_DIR}/base/logging/build
    INSTALL_DIR         ${CMAKE_INSTALL_PREFIX}
    GIT_REPOSITORY      https://github.com/rock-core/base-logging.git
    GIT_TAG             master
    CMAKE_CACHE_ARGS    -DENV{PKG_CONFIG_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/
                        -DENV{CMAKE_PREFIX_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/share/rock/cmake/
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>  -DCMAKE_BUILD_TYPE=RELEASE 
)
add_dependencies(base-logging base-cmake )

ExternalProject_Add(base-numeric
    PREFIX       base/numeric
    STAMP_DIR    ${CMAKE_CURRENT_BINARY_DIR}/base/numeric/stamp
    DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/base/numeric/src
    SOURCE_DIR   ${CMAKE_CURRENT_BINARY_DIR}/base/numeric/src
    BINARY_DIR   ${CMAKE_CURRENT_BINARY_DIR}/base/numeric/build
    INSTALL_DIR         ${CMAKE_INSTALL_PREFIX}
    GIT_REPOSITORY      https://github.com/rock-core/base-numeric.git
    GIT_TAG             master
    CMAKE_CACHE_ARGS    -DENV{PKG_CONFIG_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/
                        -DENV{CMAKE_PREFIX_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/share/rock/cmake/
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>  -DCMAKE_BUILD_TYPE=RELEASE 
)
add_dependencies(base-numeric base-types base-logging )

ExternalProject_Add(base-boost_serialization
    PREFIX       base/boost_serialization
    STAMP_DIR    ${CMAKE_CURRENT_BINARY_DIR}/base/boost_serialization/stamp
    DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/base/boost_serialization/src
    SOURCE_DIR   ${CMAKE_CURRENT_BINARY_DIR}/base/boost_serialization/src
    BINARY_DIR   ${CMAKE_CURRENT_BINARY_DIR}/base/boost_serialization/build
    INSTALL_DIR         ${CMAKE_INSTALL_PREFIX}
    GIT_REPOSITORY      https://github.com/envire/base-boost_serialization.git
    GIT_TAG             master
    CMAKE_CACHE_ARGS    -DENV{PKG_CONFIG_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/
                        -DENV{CMAKE_PREFIX_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/share/rock/cmake/
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>  -DCMAKE_BUILD_TYPE=RELEASE 
)
add_dependencies(base-boost_serialization base-cmake base-types base-numeric )

ExternalProject_Add(gui-osg_qt5
    PREFIX       gui/osg_qt5
    STAMP_DIR    ${CMAKE_CURRENT_BINARY_DIR}/gui/osg_qt5/stamp
    DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/gui/osg_qt5/src
    SOURCE_DIR   ${CMAKE_CURRENT_BINARY_DIR}/gui/osg_qt5/src
    BINARY_DIR   ${CMAKE_CURRENT_BINARY_DIR}/gui/osg_qt5/build
    INSTALL_DIR         ${CMAKE_INSTALL_PREFIX}
    GIT_REPOSITORY      https://github.com/rock-core/gui-osg_qt4.git
    GIT_TAG             feature/qt4-qt5
    CMAKE_CACHE_ARGS    -DENV{PKG_CONFIG_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/
                        -DENV{CMAKE_PREFIX_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/share/rock/cmake/
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>  -DCMAKE_BUILD_TYPE=RELEASE 
)
add_dependencies(gui-osg_qt5 base-cmake )

ExternalProject_Add(gui-qtpropertybrowser
    PREFIX       gui/qtpropertybrowser
    STAMP_DIR    ${CMAKE_CURRENT_BINARY_DIR}/gui/qtpropertybrowser/stamp
    DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/gui/qtpropertybrowser/src
    SOURCE_DIR   ${CMAKE_CURRENT_BINARY_DIR}/gui/qtpropertybrowser/src
    BINARY_DIR   ${CMAKE_CURRENT_BINARY_DIR}/gui/qtpropertybrowser/build
    INSTALL_DIR         ${CMAKE_INSTALL_PREFIX}
    GIT_REPOSITORY      https://github.com/abhijitkundu/QtPropertyBrowser.git
    GIT_TAG             master
    CMAKE_CACHE_ARGS    -DENV{PKG_CONFIG_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/
                        -DENV{CMAKE_PREFIX_PATH}:STRING=${CMAKE_INSTALL_PREFIX}/share/rock/cmake/
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>  -DCMAKE_BUILD_TYPE=RELEASE 
)

configure_file(env.sh.in ${PROJECT_SOURCE_DIR}/env.sh)
install(FILES ${PROJECT_SOURCE_DIR}/env.sh DESTINATION ${CMAKE_INSTALL_PREFIX})
