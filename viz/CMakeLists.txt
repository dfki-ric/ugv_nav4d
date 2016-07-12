rock_vizkit_plugin(ugv_nav4d-viz
    EnvironmentXYZThetaVisualization.cpp
    DEPS ugv_nav4d
    DEPS_PKGCONFIG osgViz
    DEPS_PKGCONFIG PrimitivesFactory
    MOC EnvironmentXYZThetaVisualization.hpp
    HEADERS EnvironmentXYZThetaVisualization.hpp)

