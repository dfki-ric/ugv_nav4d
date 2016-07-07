#include "EnvironmentXYZThetaVisualization.hpp"
#include <osg/ShapeDrawable>
#include <osgViz/OsgViz.hpp>

using namespace vizkit3d;
using namespace osg;

struct EnvironmentXYZThetaVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    EnvironmentXYZTheta data;
    ref_ptr<osgviz::Object> root;
    double gridSize;
    Vec3d startPos;
    Vec3d goalPos;
    std::vector<QVector3D> solutionPath;
    
};


EnvironmentXYZThetaVisualization::EnvironmentXYZThetaVisualization()
    : p(new Data)
{
    p->root = new osgviz::Object();
}

EnvironmentXYZThetaVisualization::~EnvironmentXYZThetaVisualization()
{
    delete p;
}

ref_ptr<Node> EnvironmentXYZThetaVisualization::createMainNode()
{
    return p->root;
}

void EnvironmentXYZThetaVisualization::updateMainNode ( Node* node )
{
    Box* unitCube = new Box( Vec3(0,0,0), p->gridSize);
    ShapeDrawable* unitCubeDrawable = new ShapeDrawable(unitCube);
    
//     for(const maps::grid::Vector3d& pos : p->data.debugRobotPositions)
//     {
//         PositionAttitudeTransform* trans = new PositionAttitudeTransform();
//         const Vec3d osgPos(pos.x(), pos.y(), pos.z());
//         trans->setPosition(osgPos);
//         p->root->addChild(trans);
//         Geode* childGeode = new Geode();
//         childGeode->addDrawable(unitCubeDrawable);
//         trans->addChild(childGeode);
//     }

    for(const QVector3D& pos : p->solutionPath)
    {
        PositionAttitudeTransform* trans = new PositionAttitudeTransform();
        const Vec3d osgPos(pos.x(), pos.y(), pos.z());
        trans->setPosition(osgPos);
        p->root->addChild(trans);
        Geode* childGeode = new Geode();
        childGeode->addDrawable(unitCubeDrawable);
        trans->addChild(childGeode);
    }
    
    // Update the main node using the data in p->data
}

void EnvironmentXYZThetaVisualization::updateDataIntern(EnvironmentXYZTheta const& value)
{
    p->data = value;
}

void EnvironmentXYZThetaVisualization::setGridSize(const double gridSize)
{
    p->gridSize = gridSize;
}

void EnvironmentXYZThetaVisualization::setGoalPos(const double x, const double y, const double z)
{
    p->goalPos = Vec3d(x, y, z);
}

void EnvironmentXYZThetaVisualization::setStartPos(const double x, const double y, const double z)
{
    p->startPos = Vec3d(x, y, z);
}

void EnvironmentXYZThetaVisualization::setSolution(std::vector< QVector3D > path)
{
    p->solutionPath = path;
}



//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(EnvironmentXYZThetaVisualization)

