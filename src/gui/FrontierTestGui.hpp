#pragma once
#include <QObject>
#include <memory>
#ifndef Q_MOC_RUN
#include <base/Eigen.hpp>
#include <vizkit3d/MLSMapVisualization.hpp>
#include <vizkit3d/TraversabilityMap3dVisualization.hpp>
#include <ugv_nav4d/FrontierGenerator.hpp>
#include <ugv_nav4d/AreaExplorer.hpp>
#endif

namespace vizkit3d 
{
class Vizkit3DWidget;
}
class QProgressBar;

namespace ugv_nav4d
{

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
    void distToGoalFactorEditFinished();
    void distFromStartFactorEditFinished();
    void explorableFactorSpinBoxEditFinished();
    void generateFrontier();
    
    //is used for all 3 box sizes, thus param is ignored
    void boxSizeChanged(double);
    void boxRotChanged(double);

    
private:
    
    base::Orientation getBoxOrientation() const;
    base::Vector3d getBoxSize() const;
    
    vizkit3d::Vizkit3DWidget* widget;
    QWidget* buttonWidget;
    vizkit3d::MLSMapVisualization mlsViz;
    vizkit3d::TraversabilityMap3dVisualization travViz;
    
    std::shared_ptr<FrontierGenerator> frontGen;
    std::shared_ptr<AreaExplorer> areaExplorer;
    base::Vector3d robotPos;
    base::Vector3d goalPos;
    base::Quaterniond boxOrientation;
    FrontierGeneratorParameters costParams;
    QDoubleSpinBox* distToGoalFactorSpinBox;
    QDoubleSpinBox* distFromStartFactorSpinBox;
    QDoubleSpinBox* explorableFactorSpinBox;
    QDoubleSpinBox* xSpinbox;
    QDoubleSpinBox* ySpinbox;
    QDoubleSpinBox* zSpinbox;
    QDoubleSpinBox* xOrientationSpinbox;
    QDoubleSpinBox* yOrientationSpinbox;
    QDoubleSpinBox* zOrientationSpinbox;
    double res;
};
}
