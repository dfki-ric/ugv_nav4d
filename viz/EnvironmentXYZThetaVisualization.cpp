#include "EnvironmentXYZThetaVisualization.hpp"
#include <osg/ShapeDrawable>
#include <osgViz/OsgViz.hpp>
#include <osg/PolygonMode>
#include <osg/Material>
#include <osgViz/modules/viz/Primitives/PrimitivesFactory.h>
#include <vizkit3d/ColorConversionHelper.hpp>

using namespace vizkit3d;
using namespace osg;

// TODO These helper functions should be moved to some common header
template <class T>
osg::Vec3 vec3( const Eigen::MatrixBase<T>& v )
{
    assert(v.size()==3 && "Must pass a 3x1 vector");
    return osg::Vec3( v.x(), v.y(), v.z() );
}

template<class T>
osg::Quat quat( const Eigen::QuaternionBase<T>& q)
{
    return osg::Quat(q.x(), q.y(), q.z(), q.w());
}

void setColor(const osg::Vec4d& color, osg::Geode* geode)
{
    osg::Material *material = new osg::Material();
    material->setDiffuse(osg::Material::FRONT,  osg::Vec4(0.1, 0.1, 0.1, 1.0));
    material->setSpecular(osg::Material::FRONT, osg::Vec4(0.6, 0.6, 0.6, 1.0));
    material->setAmbient(osg::Material::FRONT,  osg::Vec4(0.1, 0.1, 0.1, 1.0));
    material->setEmission(osg::Material::FRONT, color);
    material->setShininess(osg::Material::FRONT, 10.0);

    geode->getOrCreateStateSet()->setAttribute(material);
}

void addShape(osg::Group* group, osg::ref_ptr<osg::Shape> s, const osg::Vec4f& color,
              bool wireframe = false)
{
    ref_ptr<Geode> geo = new Geode();
    ref_ptr<ShapeDrawable> sd = new ShapeDrawable(s);
    sd->setColor(color);
    
    if(wireframe)
    {
        osg::ref_ptr<osg::StateSet> stateset = new osg::StateSet;
        osg::ref_ptr<osg::PolygonMode> polymode = new osg::PolygonMode;
        polymode->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE);
        stateset->setAttributeAndModes(polymode,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
        sd->setStateSet(stateset);
    }
    geo->addDrawable(sd);
//    setColor(color, geo);
    group->addChild(geo);
}


struct EnvironmentXYZThetaVisualization::Data {
    Eigen::Vector3d robotSize2;
    std::vector<base::Pose> collisionPoses;
    std::vector<Eigen::Vector3d> succs;
    
    //[0..2] = x,y,z, [3] = cost
    std::vector<Eigen::Vector4d> cost;

    ref_ptr<osgviz::Object> root;
    double gridSize;
    Vec3d startPos;
    Vec3d goalPos;
};


EnvironmentXYZThetaVisualization::EnvironmentXYZThetaVisualization()
    : p(new Data)
{
    p->root = new osgviz::Object();
    p->gridSize = 0.1;
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
    
    osg::Group* group = node->asGroup();
    group->removeChildren(0, node->asGroup()->getNumChildren());

    if(showHeuristic)
    {
        Box* costBox = new Box(Vec3(0,0,0), 0.1);
        costBox->setHalfLengths(osg::Vec3(0.05, 0.05, 0.05));
        const double costMax = 100; //FIXME because ransac sometimes produces extrem heights
        
        for(const Eigen::Vector4d& cost : p->cost)
        { 
            ShapeDrawable* costDrawable = new ShapeDrawable(costBox);
            costDrawable->setColor(osg::Vec4(1, cost[3]/costMax, 0, 0.5));
            PositionAttitudeTransform* trans = new PositionAttitudeTransform();
            const Vec3d osgPos(cost.x(), cost.y(), cost.z());
            trans->setPosition(osgPos);
            p->root->addChild(trans);
            Geode* childGeode = new Geode();
            childGeode->addDrawable(costDrawable);
            trans->addChild(childGeode);
        }
    }
    
    Box* succBox = new Box(Vec3(0,0,0), 0.05);
    succBox->setHalfLengths(osg::Vec3(0.05, 0.05, 0.05));
    const int end = std::min(numSuccs, int(p->succs.size()));
    for(int i = 0; i < end; ++i)
    {  
        const Eigen::Vector3d succ = p->succs[i];
        ShapeDrawable* costDrawable = new ShapeDrawable(succBox);
        costDrawable->setColor(osg::Vec4(double(i)/double(p->succs.size()), 0, 1, 0.3));
        PositionAttitudeTransform* trans = new PositionAttitudeTransform();
        const Vec3d osgPos(succ.x(), succ.y(), succ.z());
        trans->setPosition(osgPos);
        p->root->addChild(trans);
        Geode* childGeode = new Geode();
        childGeode->addDrawable(costDrawable);
        trans->addChild(childGeode);
    }
    
    if(showCollisions)
    {
        const int maxPoses = 500; //to avoid lag
        int step = p->collisionPoses.size() / maxPoses;
        if(step <= 0)
            step = 1; 
        for(size_t i = 0; i < p->collisionPoses.size(); i += step)
        {
            const auto pose = p->collisionPoses[i];
            osg::ref_ptr<osg::Box> box(new osg::Box);
            box->setHalfLengths(vec3(p->robotSize2));
            box->setRotation(quat(pose.orientation));
            box->setCenter(vec3(pose.position));
            addShape(p->root, box, osg::Vec4f(1.0f, 0.1f, 0.1f, 1.0f), true);
        }
    }
}

void EnvironmentXYZThetaVisualization::updateDataIntern(ugv_nav4d::EnvironmentXYZTheta const& value)
{
    p->robotSize2 = value.robotHalfSize;
    p->collisionPoses = value.debugCollisionPoses;
}

void EnvironmentXYZThetaVisualization::setGridSize(const double gridSize)
{
    p->gridSize = gridSize;
    setDirty();
}

void EnvironmentXYZThetaVisualization::setGoalPos(const double x, const double y, const double z)
{
    p->goalPos = Vec3d(x, y, z);
    setDirty();
}

void EnvironmentXYZThetaVisualization::setStartPos(const double x, const double y, const double z)
{
    p->startPos = Vec3d(x, y, z);
    setDirty();
}


void EnvironmentXYZThetaVisualization::setHeuristic(const std::vector<Eigen::Vector4d>& cost)
{
    p->cost = cost;
}

void EnvironmentXYZThetaVisualization::setCollisionPoses(std::vector<base::Pose>& poses)
{
    p->collisionPoses = poses;
}

void EnvironmentXYZThetaVisualization::setRobotHalfSize(const Eigen::Vector3d& value)
{
    p->robotSize2 = value;
}

void EnvironmentXYZThetaVisualization::setSuccessors(std::vector<Eigen::Vector3d>& succs)
{
    p->succs = succs;
}

int EnvironmentXYZThetaVisualization::getNumSuccs()
{
    return numSuccs;
}

void EnvironmentXYZThetaVisualization::setNumSuccs(int val)
{
    numSuccs = val;
    emit propertyChanged("numSuccs");
    setDirty();
}

bool EnvironmentXYZThetaVisualization::getShowHeuristic()
{
    return showHeuristic;
}

void EnvironmentXYZThetaVisualization::setShowHeuristic(bool val)
{
    showHeuristic = val;
    emit propertyChanged("showHeuristic");
    setDirty();
}

bool EnvironmentXYZThetaVisualization::getShowCollisions()
{
    return showCollisions;
}

void EnvironmentXYZThetaVisualization::setShowCollisions(bool val)
{
    showCollisions = val;
    emit propertyChanged("showCollisions");
    setDirty();
}






//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(EnvironmentXYZThetaVisualization)

