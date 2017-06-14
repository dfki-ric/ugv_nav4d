#pragma once
#include <QApplication>
#include <QObject>
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d/MLSMapVisualization.hpp>
#include <maps/grid/MLSMap.hpp>

class MinimalClickGui : public QObject
{
    Q_OBJECT;
    
public:
    MinimalClickGui(int argc, char** argv);
    
    void show();
public slots:
    /** Called when the user clicks a patch on the mls */
    void picked(float x, float y,float z, int buttonMask, int modifierMask);
    void loadPly(const std::string& path);
        
private:
    
    
    vizkit3d::Vizkit3DWidget* widget;
    vizkit3d::MLSMapVisualization mlsViz;
};





  
