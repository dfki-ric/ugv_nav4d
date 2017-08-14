#include "FrontierTestGui.hpp"
#include "FrontierGenerator.hpp"
#include <vizkit3d/Vizkit3DWidget.hpp>
#include <vizkit3d_debug_drawings/DebugDrawing.h>
#include <vizkit3d_debug_drawings/DebugDrawingColors.h>
#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>
#include <thread>

namespace ugv_nav4d
{

FrontierTestGui::FrontierTestGui(int argc, char** argv)
{
    double res = 0.2; //FIXME parse from argc
    TraversabilityConfig conf;
    conf.gridResolution = res;
    conf.maxSlope = 0.58; //40.0/180.0 * M_PI;
    conf.maxStepHeight = 0.5; //space below robot
    conf.robotSizeX = 1.0;
    conf.robotSizeY =  0.7;
    conf.robotHeight = 0.5; //incl space below body
    conf.slopeMetricScale = 0.0;
    conf.slopeMetric = SlopeMetric::NONE;
    conf.heuristicType = HeuristicType::HEURISTIC_2D;
    conf.inclineLimittingMinSlope = 0.35; // 10.0 * M_PI/180.0;
    conf.inclineLimittingLimit = 0.44;// 5.0 * M_PI/180.0;
    conf.parallelismEnabled = false;
    conf.costFunctionDist = 0.4;
    
    frontGen.reset(new FrontierGenerator(conf, costParams));
    areaExplorer.reset(new AreaExplorer(frontGen));
    
    widget = new vizkit3d::Vizkit3DWidget();
    CONFIGURE_DEBUG_DRAWINGS_USE_EXISTING_WIDGET(widget);
    widget->setCameraManipulator(vizkit3d::ORBIT_MANIPULATOR);
    widget->addPlugin(&mlsViz);
    widget->addPlugin(&travViz);
    
    
    mlsViz.setCycleHeightColor(true);
    mlsViz.setShowPatchExtents(false); 
    mlsViz.setShowNormals(false);
    
    QDockWidget* dock = new QDockWidget();
    QVBoxLayout* layout = new QVBoxLayout();
    
    buttonWidget = new QWidget(); 

    
    QHBoxLayout* sortparamLayout = new QHBoxLayout();
    QLabel* distToGoalFactorLabel = new QLabel();
    distToGoalFactorLabel->setText("distToGoalFactor");
    sortparamLayout->addWidget(distToGoalFactorLabel);
    distToGoalFactorSpinBox = new QDoubleSpinBox();
    distToGoalFactorSpinBox->setMinimum(0);
    distToGoalFactorSpinBox->setMaximum(9999);
    distToGoalFactorSpinBox->setSingleStep(0.05);
    distToGoalFactorSpinBox->setValue(1);
    sortparamLayout->addWidget(distToGoalFactorSpinBox);
    QLabel* distFromStartFactorLabel = new QLabel();
    distFromStartFactorLabel->setText("distFromStartFactor");
    sortparamLayout->addWidget(distFromStartFactorLabel);
    distFromStartFactorSpinBox = new QDoubleSpinBox();
    distFromStartFactorSpinBox->setMinimum(0);
    distFromStartFactorSpinBox->setMaximum(9999);
    distToGoalFactorSpinBox->setSingleStep(0.05);
    distFromStartFactorSpinBox->setValue(1);
    sortparamLayout->addWidget(distFromStartFactorSpinBox);
    QLabel* explorableFactorLabel = new QLabel();
    explorableFactorLabel->setText("explorableFactor");
    sortparamLayout->addWidget(explorableFactorLabel);
    explorableFactorSpinBox = new QDoubleSpinBox();
    explorableFactorSpinBox->setMinimum(0);
    explorableFactorSpinBox->setMaximum(9999);
    distToGoalFactorSpinBox->setSingleStep(0.05);
    explorableFactorSpinBox->setValue(1);
    sortparamLayout->addWidget(explorableFactorSpinBox);
    
    connect(distToGoalFactorSpinBox, SIGNAL(editingFinished()), this, SLOT(distToGoalFactorEditFinished()));
    connect(distFromStartFactorSpinBox, SIGNAL(editingFinished()), this, SLOT(distFromStartFactorEditFinished()));
    connect(explorableFactorSpinBox, SIGNAL(editingFinished()), this, SLOT(explorableFactorSpinBoxEditFinished()));
    
    layout->addLayout(sortparamLayout);

    QHBoxLayout* boxParamslayout = new QHBoxLayout();
    QLabel* xLabel = new QLabel();
    xLabel->setText("Box x size:");
    boxParamslayout->addWidget(xLabel);
    xSpinbox = new QDoubleSpinBox();
    xSpinbox->setMinimum(0.0000001);
    xSpinbox->setMaximum(999999);
    xSpinbox->setValue(1);
    xSpinbox->setSingleStep(0.1);
    boxParamslayout->addWidget(xSpinbox);
    QLabel* yLabel = new QLabel();
    yLabel->setText("Box y size:");
    boxParamslayout->addWidget(yLabel);
    ySpinbox = new QDoubleSpinBox();
    ySpinbox->setMinimum(0.0000001);
    ySpinbox->setMaximum(999999);
    ySpinbox->setValue(1);
    ySpinbox->setSingleStep(0.1);
    boxParamslayout->addWidget(ySpinbox);
    QLabel* zLabel = new QLabel();
    zLabel->setText("Box z size:");
    boxParamslayout->addWidget(zLabel);
    zSpinbox = new QDoubleSpinBox();
    zSpinbox->setMinimum(0.0000001);
    zSpinbox->setMaximum(999999);
    zSpinbox->setValue(1);
    zSpinbox->setSingleStep(0.1);
    boxParamslayout->addWidget(zSpinbox); 
    
    QLabel* xOrientationLabel = new QLabel();
    xOrientationLabel->setText("Box x rot:");
    boxParamslayout->addWidget(xOrientationLabel);
    xOrientationSpinbox = new QDoubleSpinBox();
    xOrientationSpinbox->setMinimum(0.0000001);
    xOrientationSpinbox->setMaximum(999999);
    xOrientationSpinbox->setValue(0);
    xOrientationSpinbox->setSingleStep(0.1);
    boxParamslayout->addWidget(xOrientationSpinbox);
    QLabel* yOrientationLabel = new QLabel();
    yOrientationLabel->setText("Box y rot:");
    boxParamslayout->addWidget(yOrientationLabel);
    yOrientationSpinbox = new QDoubleSpinBox();
    yOrientationSpinbox->setMinimum(0.0000001);
    yOrientationSpinbox->setMaximum(999999);
    yOrientationSpinbox->setValue(0);
    yOrientationSpinbox->setSingleStep(0.1);
    boxParamslayout->addWidget(yOrientationSpinbox);
    QLabel* zOrientationLabel = new QLabel();
    zOrientationLabel->setText("Box z rot:");
    boxParamslayout->addWidget(zOrientationLabel);
    zOrientationSpinbox = new QDoubleSpinBox();
    zOrientationSpinbox->setMinimum(0.0000001);
    zOrientationSpinbox->setMaximum(999999);
    zOrientationSpinbox->setValue(0);
    zOrientationSpinbox->setSingleStep(0.1);
    boxParamslayout->addWidget(zOrientationSpinbox); 
    
    layout->addLayout(boxParamslayout);
    
    connect(xSpinbox, SIGNAL(valueChanged(double)), this, SLOT(boxSizeChanged(double)));
    connect(ySpinbox, SIGNAL(valueChanged(double)), this, SLOT(boxSizeChanged(double)));
    connect(zSpinbox, SIGNAL(valueChanged(double)), this, SLOT(boxSizeChanged(double)));
    connect(xOrientationSpinbox, SIGNAL(valueChanged(double)), this, SLOT(boxRotChanged(double)));
    connect(yOrientationSpinbox, SIGNAL(valueChanged(double)), this, SLOT(boxRotChanged(double)));
    connect(zOrientationSpinbox, SIGNAL(valueChanged(double)), this, SLOT(boxRotChanged(double)));   
    
    QPushButton* getFrontiersButton = new QPushButton();
    getFrontiersButton->setText("getNextFrontiers");
    layout->addWidget(getFrontiersButton);
    connect(getFrontiersButton, SIGNAL(released()), this, SLOT(getFrontiersButtonReleased()));
    
    
    buttonWidget->setLayout(layout);
    dock->setWidget(buttonWidget);
    widget->addDockWidget(Qt::BottomDockWidgetArea, dock);
    
    connect(&mlsViz, SIGNAL(picked(float,float,float, int, int)), this, SLOT(picked(float, float, float, int, int)));
    connect(this, SIGNAL(frontierCalcDone()), this, SLOT(frontierCalcIsDone()));
    
    if(argc > 1)
    {
        const std::string mls(argv[1]);
        loadMls(mls);
    }
    else
    {
        loadMls();
    }
    
}


void FrontierTestGui::picked(float x, float y,float z, int buttonMask, int modifierMask)
{
    if(buttonMask == 1) //left click
    {
        robotPos << x, y, z;
        frontGen->updateRobotPos(robotPos);
    }
    else if(buttonMask == 4) //right click
    {
        goalPos << x, y, z;
        frontGen->updateGoalPos(goalPos);
        CLEAR_DRAWING("AreaToExplore");
        DRAW_WIREFRAME_BOX("AreaToExplore", goalPos, getBoxOrientation(), getBoxSize(), vizkit3dDebugDrawings::Color::red);
    }
}

base::Orientation FrontierTestGui::getBoxOrientation() const
{
    return Eigen::AngleAxisd(xOrientationSpinbox->value(), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(yOrientationSpinbox->value(), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(zOrientationSpinbox->value(), Eigen::Vector3d::UnitZ());
}



void FrontierTestGui::show()
{
    widget->show();
}


void FrontierTestGui::getFrontiersButtonReleased()
{
    generateFrontier();
}

void FrontierTestGui::generateFrontier()
{
    OrientedBox areaToExplore(goalPos, getBoxSize(), getBoxOrientation());
    std::vector<base::samples::RigidBodyState> frontiers;
    const bool result = areaExplorer->getFrontiers(robotPos, areaToExplore, frontiers);
    
    std::cout << "RESULT = " << result << std::endl;
    travViz.updateData(frontGen->getTraversabilityBaseMap());
}

void FrontierTestGui::boxSizeChanged(double)
{
    CLEAR_DRAWING("AreaToExplore");
    DRAW_WIREFRAME_BOX("AreaToExplore", goalPos, getBoxOrientation(), getBoxSize(), vizkit3dDebugDrawings::Color::red);
}

void FrontierTestGui::boxRotChanged(double)
{
    CLEAR_DRAWING("AreaToExplore");
    DRAW_WIREFRAME_BOX("AreaToExplore", goalPos, getBoxOrientation(), getBoxSize(), vizkit3dDebugDrawings::Color::red);
}


base::Vector3d FrontierTestGui::getBoxSize() const
{
    return Eigen::Vector3d(xSpinbox->value(), ySpinbox->value(), zSpinbox->value());;
}



void FrontierTestGui::loadMls()
{
    const QString file = QFileDialog::getOpenFileName(nullptr, tr("Load mls map"),
                                                        QDir::currentPath(), QString(),
                                                        nullptr, QFileDialog::DontUseNativeDialog);
    if(!file.isEmpty())
    {
        loadMls(file.toStdString());
    }
}

void FrontierTestGui::loadMls(const std::string& path)
{
    std::ifstream fileIn(path);       
    
    if(path.find("graph_mls_kalman") != std::string::npos)
    {
        try
        {
            envire::core::EnvireGraph g;
            g.loadFromFile(path);
            maps::grid::MLSMapKalman mlsMap = (*g.getItem<envire::core::Item<maps::grid::MLSMapKalman>>("mls_map", 0)).getData();
            mlsViz.updateMLSKalman(mlsMap);
            frontGen->updateMap(mlsMap);
            return;
        }
        catch(...) {}   
    }
    else
    {
        try
        {
            boost::archive::binary_iarchive mlsIn(fileIn);
            maps::grid::MLSMapKalman mlsMap;
            mlsIn >> mlsMap;
            mlsViz.updateMLSKalman(mlsMap);
            frontGen->updateMap(mlsMap);
            return;
        }
        catch(...) {}
    }

    std::cerr << "Unabled to load mls. Unknown format" << std::endl;
    
}

void FrontierTestGui::distFromStartFactorEditFinished()
{
    costParams.distFromStartFactor = distFromStartFactorSpinBox->value();
    frontGen->updateCostParameters(costParams);
    generateFrontier();
}

void FrontierTestGui::distToGoalFactorEditFinished()
{
    costParams.distToGoalFactor = distToGoalFactorSpinBox->value();
    frontGen->updateCostParameters(costParams);
    generateFrontier();
}

void FrontierTestGui::explorableFactorSpinBoxEditFinished()
{
    costParams.explorableFactor = explorableFactorSpinBox->value();
    frontGen->updateCostParameters(costParams);
    generateFrontier();
}



}