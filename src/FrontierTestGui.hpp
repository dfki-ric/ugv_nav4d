#pragma once
#include <QObject>
#include <memory>
#include <base/Eigen.hpp>
#include <vizkit3d/MLSMapVisualization.hpp>
#include <vizkit3d/TraversabilityMap3dVisualization.hpp>
#include "FrontierGenerator.hpp"

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
    

    
private:
    vizkit3d::Vizkit3DWidget* widget;
    QWidget* buttonWidget;
    vizkit3d::MLSMapVisualization mlsViz;
    vizkit3d::TraversabilityMap3dVisualization travViz;
    
    std::shared_ptr<FrontierGenerator> frontGen;
    base::Vector3d robotPos;
    base::Vector3d goalPos;
    CostFunctionParameters costParams;
    QDoubleSpinBox* distToGoalFactorSpinBox;
    QDoubleSpinBox* distFromStartFactorSpinBox;
    QDoubleSpinBox* explorableFactorSpinBox;
};
}