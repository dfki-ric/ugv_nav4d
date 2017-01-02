#include "EnvironmentXYZThetaVisualization.hpp"
#include <osg/ShapeDrawable>
#include <osgViz/OsgViz.hpp>
#include <osg/PolygonMode>
#include <osg/Material>
#include <osgViz/modules/viz/Primitives/PrimitivesFactory.h>
#include <osgViz/modules/viz/Primitives/Primitives/LinesNode.h>
#include <vizkit3d/ColorConversionHelper.hpp>
#include <QString>

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
    ref_ptr<osgviz::Object> root;
    double gridSize;    
    ugv_nav4d_debug::EnvironmentXYZThetaDebugData envDebug;
    ugv_nav4d_debug::TravGenDebugData travGenDebugData;
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
        std::vector<Eigen::Vector4d> costs = p->envDebug.getHeuristicCostForViz();
        for(const Eigen::Vector4d& cost : costs)
        { 
            osgText::Text *text= new ::osgText::Text;
            text->setText(QString::number(cost[3], 'f', 3).toStdString());
            text->setCharacterSize(0.1/4.0);
            text->setAxisAlignment(osgText::Text::XY_PLANE);
            text->setColor(osg::Vec4(0.9f, 0.1f, 0.1f, 1.0f));
            text->setPosition(osg::Vec3(0, 0, 0.05));
            PositionAttitudeTransform* trans = new PositionAttitudeTransform();
            const Vec3d osgPos(cost.x(), cost.y(), cost.z());
            trans->setPosition(osgPos);
            p->root->addChild(trans);
            Geode* childGeode = new Geode();
            childGeode->addDrawable(text);
            trans->addChild(childGeode);
        }
    }
    
    if(showSlopes)
    {       
        for(const Eigen::Vector4d& slope : p->travGenDebugData.getSlopes())
        { 
            osgText::Text *text= new ::osgText::Text;
            text->setText(QString::number(slope[3], 'f', 3).toStdString());
            text->setCharacterSize(0.1/4.0);
            text->setAxisAlignment(osgText::Text::XY_PLANE);
            text->setColor(osg::Vec4(0.1f, 0.1f, 0.9f, 1.0f));
            text->setPosition(osg::Vec3(0, 0, 0.05));
            PositionAttitudeTransform* trans = new PositionAttitudeTransform();
            const Vec3d osgPos(slope.x(), slope.y(), slope.z());
            trans->setPosition(osgPos);
            p->root->addChild(trans);
            Geode* childGeode = new Geode();
            childGeode->addDrawable(text);
            trans->addChild(childGeode);
        }
        
        osgviz::LinesNode* slopeLines = new osgviz::LinesNode(osg::Vec4(1, 0, 0, 1));
        for(const Eigen::Matrix<double, 2, 3>& slopeDir : p->travGenDebugData.getSlopeDirs())
        {
            const Eigen::Vector3d pos = slopeDir.row(0);
            const Eigen::Vector3d dir = slopeDir.row(1) * p->gridSize;
            const osg::Vec3 start(pos.x(), pos.y(), pos.z());
            const osg::Vec3 end(pos.x() + dir.x(), pos.y() + dir.y(), pos.z() + dir.z());
            slopeLines->addLine(start, end);          
        }
        p->root->addChild(slopeLines);
    }
    
    if(showAllowedSlopes)
    {
        osgviz::LinesNode* slopeLines = new osgviz::LinesNode(osg::Vec4(1, 1, 0, 1));
//         std::cout << "SLOPE DEBUG: " << p->slopeDebug.size() << std::endl;
//         std::cout << "SLOPE CAND DEBUG: " << p->slopeDebugCandidates.size() << std::endl;
        for(const auto& data : p->envDebug.getSlopeData())
        {
            const osg::Vec3 start(data.start.x(), data.start.y(), data.start.z() + 0.05);            
            const osg::Vec3 end1(data.end1.x(), data.end1.y(), data.end1.z() + 0.05);
            const osg::Vec3 end2(data.end2.x(), data.end2.y(), data.end2.z() + 0.05);
            const osg::Vec3 end3(data.end3.x(), data.end3.y(), data.end3.z() + 0.05);
            const osg::Vec3 end4(data.end4.x(), data.end4.y(), data.end4.z() + 0.05);
        
            slopeLines->addLine(start, end1);
            slopeLines->addLine(start, end2);
            slopeLines->addLine(start, end3);
            slopeLines->addLine(start, end4);
        }
        p->root->addChild(slopeLines);
        
        osgviz::LinesNode* redLines = new osgviz::LinesNode(osg::Vec4(1, 0, 0, 1));
        osgviz::LinesNode* greenLines = new osgviz::LinesNode(osg::Vec4(0, 1, 0, 1));
        for(const auto& candidate : p->envDebug.getSlopeCandidates())
        {
            
            const osg::Vec3 start(candidate.start.x(), candidate.start.y(), candidate.start.z() + 0.05);
            const osg::Vec3 end(candidate.end.x(), candidate.end.y(), candidate.end.z() + 0.05);

            if(candidate.color == ugv_nav4d_debug::DebugSlopeCandidate::RED)
                redLines->addLine(start, end);
            else if(candidate.color == ugv_nav4d_debug::DebugSlopeCandidate::GREEN)
                greenLines->addLine(start, end);
            else
                std::cout << "UNKNOWN COLOR ERROR!!! ARRRR" << std::endl;
        }
        p->root->addChild(redLines);
        p->root->addChild(greenLines);
        
    }
    

    Box* succBox = new Box(Vec3(0,0,0), 0.05);
    succBox->setHalfLengths(osg::Vec3(0.05, 0.05, 0.05));
    const int end = std::min(numSuccs, int(p->envDebug.getSuccs().size()));
    for(int i = 0; i < end; ++i)
    {  
        const Eigen::Vector3d succ = p->envDebug.getSuccs()[i];
        ShapeDrawable* costDrawable = new ShapeDrawable(succBox);
        costDrawable->setColor(osg::Vec4(double(i)/double(p->envDebug.getSuccs().size()), 0, 1, 0.3));
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
        int step = p->envDebug.getCollisions().size() / maxPoses;
        if(step <= 0)
            step = 1; 
        for(size_t i = 0; i < p->envDebug.getCollisions().size(); i += step)
        {
            const auto pose = p->envDebug.getCollisions()[i];
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
}

void EnvironmentXYZThetaVisualization::setGridSize(const double gridSize)
{
    p->gridSize = gridSize;
    setDirty();
}

void EnvironmentXYZThetaVisualization::setRobotHalfSize(const Eigen::Vector3d& value)
{
    p->robotSize2 = value;
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

bool EnvironmentXYZThetaVisualization::getShowSlopes()
{
    return showSlopes;
}

bool EnvironmentXYZThetaVisualization::getshowAllowedSlopes()
{
    return showAllowedSlopes;
}

void EnvironmentXYZThetaVisualization::setshowAllowedSlopes(bool val)
{
    showAllowedSlopes = val;
}

void EnvironmentXYZThetaVisualization::setShowSlopes(bool val)
{
    showSlopes = val;
    emit propertyChanged("showSlopes");
    setDirty();
}

void EnvironmentXYZThetaVisualization::setEnvDebugData(const ugv_nav4d_debug::EnvironmentXYZThetaDebugData& data)
{
    p->envDebug = data;
}

void EnvironmentXYZThetaVisualization::setTravGenDebugData(const ugv_nav4d_debug::TravGenDebugData& data)
{
    p->travGenDebugData = data;
}




//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(EnvironmentXYZThetaVisualization)

