
#include "MinimalClickGui.hpp"
#ifndef Q_MOC_RUN
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>
#include <QFileDialog>
#include <iostream>
#include <maps/grid/MLSConfig.hpp>
#include <maps/grid/MLSMap.hpp>
#include <base/samples/Pointcloud.hpp>
#include <pcl/pcl_config.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#endif

MinimalClickGui::MinimalClickGui(int argc, char** argv): QObject()
{
    widget = new vizkit3d::Vizkit3DWidget();
    V3DD::CONFIGURE_DEBUG_DRAWINGS_USE_EXISTING_WIDGET(widget);
    
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PLYReader plyReader;
        if(plyReader.read(path, *cloud) >= 0)
        {
            pcl::PointXYZ mi, ma; 
            pcl::getMinMax3D (*cloud, mi, ma); 
            
            double mls_res = 0.2;
            double size_x = std::max(ma.x, -std::min<float>(mi.x, 0.0)) * 2.0;
            double size_y = std::max(ma.y, -std::min<float>(mi.y, 0.0)) * 2.0;
            
            std::cout << "MIN: " << mi << ", MAX: " << ma << std::endl;
            maps::grid::MLSConfig cfg;
            maps::grid::MLSMapKalman map(maps::grid::Vector2ui(size_x / mls_res, size_y / mls_res), maps::grid::Vector2d(mls_res, mls_res), cfg);
            map.translate(base::Vector3d(- size_x / 2.0, - size_y / 2.0, 0));
            base::Transform3d tf = base::Transform3d::Identity();
            map.mergePointCloud(*cloud, tf, 0.01);
            
            mlsViz.updateMLSKalman(map);
        }
    }
}

void MinimalClickGui::show()
{
    widget->show();
}


