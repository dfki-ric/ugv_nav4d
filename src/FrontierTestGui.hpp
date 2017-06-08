#pragma once
#include <QObject>
#include <memory>
#include <base/Eigen.hpp>
#include <vizkit3d/MLSMapVisualization.hpp>

namespace vizkit3d 
{
class Vizkit3DWidget;
}
class QProgressBar;

namespace ugv_nav4d
{
class FrontierGenerator;

class FrontierTestGui : public QObject
{
    Q_OBJECT;
    
public: 
    FrontierTestGui(int argc, char** argv);
    
    void show();
public slots:
    /** Called when the user clicks a patch on the mls */
    void picked(float x, float y,float z, int buttonMask, int modifierMask);

    void loadMls();
    void loadMls(const std::string& path);
    
private slots:
    void getFrontiersButtonReleased();
    void frontierCalcIsDone();
    
signals:
    void frontierCalcDone();

    
private:
    vizkit3d::Vizkit3DWidget* widget;
    QWidget* buttonWidget;
    QProgressBar* bar;
    vizkit3d::MLSMapVisualization mlsViz;
    
    std::shared_ptr<FrontierGenerator> frontGen;
    base::Vector3d robotPos;
    base::Vector3d goalPos;
};
}