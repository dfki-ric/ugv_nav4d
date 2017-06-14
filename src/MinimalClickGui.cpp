#include "MinimalClickGui.hpp"
#include <vizkit3d_debug_drawings/DebugDrawing.h>
#include <QFileDialog>
#include <iostream>
#include <envire_core/graph/EnvireGraph.hpp>
#include <maps/grid/MLSMap.hpp>
#include <base/samples/Pointcloud.hpp>
#include <pcl/pcl_config.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>


MinimalClickGui::MinimalClickGui(int argc, char** argv): QObject()
{
    widget = new vizkit3d::Vizkit3DWidget();
    CONFIGURE_DEBUG_DRAWINGS_USE_EXISTING_WIDGET(widget);
    
    connect(&mlsViz, SIGNAL(picked(float,float,float, int, int)), this, SLOT(picked(float, float, float, int, int)));
    
    widget->addPlugin(&mlsViz);

    const QString file = QFileDialog::getOpenFileName(nullptr, tr("Load ply "),
                                                        QDir::currentPath(), QString(),
                                                        nullptr, QFileDialog::DontUseNativeDialog);
    if(!file.isEmpty())
    {
        loadPly(file.toStdString());
    }
    
}

void MinimalClickGui::picked(float x, float y, float z, int buttonMask, int modifierMask)
{
    std::cout << "PICKED: " << x  << ", " << y << ", " << z << ", button: " << buttonMask << ", mod: " << modifierMask << std::endl;
}


void MinimalClickGui::loadPly(const std::string& path)
{
    if(!path.empty())
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PLYReader plyReader;
        if(plyReader.read(path, cloud) >= 0)
        {
            //FIXME choose params and tf
            maps::grid::MLSMapKalman map;
            map.setResolution(maps::grid::Vector2d(0.01, 0.01));
            map.resize(maps::grid::Vector2ui(5000, 5000));
            base::TransformWithCovariance tf;
            tf.translation << 0, 0, 0;
            tf.orientation = base::Quaterniond::Identity();
            map.mergePointCloud(cloud, tf);
            mlsViz.updateMLSKalman(map);
        }
    }
}

void MinimalClickGui::show()
{
    widget->show();
}


