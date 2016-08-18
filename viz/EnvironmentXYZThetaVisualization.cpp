#include "EnvironmentXYZThetaVisualization.hpp"
#include <osg/ShapeDrawable>
#include <osgViz/OsgViz.hpp>
#include <osg/PolygonMode>
#include <osgViz/plugins/viz/Primitives/PrimitivesFactory.h>
#include <vizkit3d/ColorConversionHelper.hpp>

using namespace vizkit3d;
using namespace osg;

struct EnvironmentXYZThetaVisualization::Data {
    
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    //FIXME remove all debug code afterwards
    std::vector<maps::grid::Vector3d> debugRobotPositions;
    //contains the min/max vectors for bounding boxes that collided with something
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> debugCollisions;
    
    std::vector<Eigen::Matrix<double, 3, 8>> debugRotatedCorners;
    
    std::vector<Eigen::Vector3d> intersectionPositions;

    ref_ptr<osgviz::Object> root;
    double gridSize;
    Vec3d startPos;
    Vec3d goalPos;
    std::vector<QVector3D> solutionPath;
    std::vector<ugv_nav4d::Motion> solutionMotions;
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
    std::cout << "AAAAA" << std::endl;
     Box* unitCube = new Box( Vec3(0,0,0), p->gridSize);
//     ShapeDrawable* unitCubeDrawable = new ShapeDrawable(unitCube);
//     unitCubeDrawable->setColor(osg::Vec4(1, 0, 0, 1));
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
    
    ShapeDrawable* unitCubeDrawable2 = new ShapeDrawable(unitCube);
    unitCubeDrawable2->setColor(osg::Vec4(0, 1, 0, 1));
    for(const QVector3D& pos : p->solutionPath)
    {
        PositionAttitudeTransform* trans = new PositionAttitudeTransform();
        const Vec3d osgPos(pos.x(), pos.y(), pos.z());
        trans->setPosition(osgPos);
        p->root->addChild(trans);
        Geode* childGeode = new Geode();
        childGeode->addDrawable(unitCubeDrawable2);
        trans->addChild(childGeode);
    }

//     Geode* collisionGeode = new Geode();
    osgviz::PrimitivesFactory fac(nullptr);
//     for(const std::pair<Eigen::Vector3d, Eigen::Vector3d>& aabb : p->debugCollisions)
//     {
//         const double xSize = aabb.second.x() - aabb.first.x();
//         const double ySize = aabb.second.y() - aabb.first.y();
//         const double zSize = aabb.second.z() - aabb.first.z();
//         osg::ref_ptr<Node> box = fac.createWireframeBox(xSize, ySize, zSize);
//         osg::PositionAttitudeTransform* trans = new osg::PositionAttitudeTransform();
//         trans->setPosition(osg::Vec3d(aabb.first.x() + (xSize/2.0),
//                                       aabb.first.y() + (ySize/2.0),
//                                       aabb.first.z() + (zSize/2.0)));
//         trans->addChild(box);
//         p->root->addChild(trans);
//     }
    
    for(const QVector3D& pos : p->solutionPath)
    {
        PositionAttitudeTransform* trans = new PositionAttitudeTransform();
        const Vec3d osgPos(pos.x(), pos.y(), pos.z());
        trans->setPosition(osgPos);
        p->root->addChild(trans);
        Geode* childGeode = new Geode();
        childGeode->addDrawable(unitCubeDrawable2);
        trans->addChild(childGeode);
    }
    for(const Eigen::Matrix<double, 3, 8>& corners : p->debugRotatedCorners)
    {
        osg::Geode* geode = new osg::Geode();
        osg::Geometry* myBox = new osg::Geometry();
        geode->addDrawable(myBox);
        osg::Vec3Array* vertices = new osg::Vec3Array;
        for(int i = 0; i < 8; ++i)
        {
            vertices->push_back(osg::Vec3(corners.col(i)(0), corners.col(i)(1),
                                          corners.col(i)(2)));
        }
        
        myBox->setVertexArray(vertices);
        unsigned short bottomLines[8] = {0, 1, 1, 2, 2, 3, 3, 0};
        myBox->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::LINES, 8, bottomLines));
        unsigned short topLines[8] = {4, 5, 5, 6, 6, 7, 7, 4};
        myBox->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::LINES, 8, topLines));        
        unsigned short sideALines[8] = {0, 1, 1, 4, 4, 7, 7, 0};
        myBox->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::LINES, 8, sideALines));        
        unsigned short sideBLines[8] = {1, 1, 2, 4, 4, 5, 5, 1};
        myBox->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::LINES, 8, sideBLines));        
        unsigned short sideCLines[8] = {2, 3, 3, 5, 5, 6, 6, 2};
        myBox->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::LINES, 8, sideCLines));        
        unsigned short sideDLines[8] = {0, 3, 3, 6, 6, 7, 7, 0};
        myBox->addPrimitiveSet(new osg::DrawElementsUShort(osg::PrimitiveSet::LINES, 8, sideDLines));        
        
        osg::Vec4Array* colors = new osg::Vec4Array;
        colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
        myBox->setColorArray(colors);
        myBox->setColorBinding(Geometry::BIND_OVERALL);
        
        p->root->addChild(geode);
    }
    
    for(const Eigen::Vector3d& intersection : p->intersectionPositions)
    {
        PositionAttitudeTransform* trans = new PositionAttitudeTransform();
        const Vec3d osgPos(intersection.x(), intersection.y(), intersection.z());
        trans->setPosition(osgPos);
        p->root->addChild(trans);
        
        osg::Sphere* s = new osg::Sphere(osg::Vec3d(0, 0, 0), 0.01);
        
        Geode* childGeode = new Geode();
        childGeode->addDrawable(new osg::ShapeDrawable(s));
        trans->addChild(childGeode);
    }
    
    osg::Vec3 startPos(0, 0, 0);
    double hue = 0.0;
    float r, g, b;
    const double hue_step = 0.35;
    for(ugv_nav4d::Motion& motion : p->solutionMotions)
    {  
        osg::Geode* geode = new osg::Geode();
        osg::Geometry* line = new osg::Geometry();
        geode->addDrawable(line);
        osg::Vec3Array* vertices = new osg::Vec3Array;
        vertices->push_back(startPos);
        for(const ugv_nav4d::PoseWithCell& pose : motion.intermediateSteps)
        {
            vertices->push_back(osg::Vec3(pose.pose.position.x() + startPos.x(), pose.pose.position.y() + startPos.y(), 0));
        }
        startPos.x() += motion.intermediateSteps.back().pose.position.x();
        startPos.y() += motion.intermediateSteps.back().pose.position.y();
        line->setVertexArray(vertices);
        line->addPrimitiveSet(new osg::DrawArrays(GL_LINE_STRIP,0,vertices->size())); 
        osg::Vec4Array* colors = new osg::Vec4Array;
        vizkit3d::hslToRgb(hue, 1.0, 0.5, r, g, b);
        hue += hue_step;
        if(hue >= 1.0) hue = 0.0;
        colors->push_back(osg::Vec4(r, g, b, 1.0f));
        line->setColorArray(colors);
        line->setColorBinding(Geometry::BIND_OVERALL);
        p->root->addChild(geode);
    }
          
//          const double robotSizeX = 0.5;
//          const double robotSizeY = 0.8;
//          const double robotSizeZ = 0.2;  
//         const double x2 = robotSizeX / 2.0;
//         const double y2 = robotSizeY / 2.0;
//         const double z2 = robotSizeZ / 2.0;
//         const double zRot = 0.473; //FIXME get real value later
//         //TODO improve performance by pre calculating lots of stuff
//         //create rotated robot bounding box
//         Eigen::Matrix<double, 3, 8> corners; //colwise corner vectors
//         corners.col(0) << -x2, -y2, -z2;
//         corners.col(1) << x2, -y2, -z2;
//         corners.col(2) << x2, y2, -z2;
//         corners.col(3) << -x2, y2, -z2;
//         corners.col(4) << x2, -y2, z2;
//         corners.col(5) << x2, y2, z2;
//         corners.col(6) << -x2, y2, z2;
//         corners.col(7) << -x2, -y2, z2;
// 
//         const Eigen::Matrix3d rot = Eigen::AngleAxisd(zRot, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
//         const Eigen::Matrix<double, 3, 8> rotatedCorners = rot * corners;
//         //find min/max for bounding box
//         const Eigen::Vector3d min = rotatedCorners.rowwise().minCoeff();
//         const Eigen::Vector3d max = rotatedCorners.rowwise().maxCoeff();
//         
//         const Eigen::AlignedBox3d aabb(min, max); //aabb around the rotated robot bounding box    
//     
//     
//         osgviz::PrimitivesFactory fac(nullptr);
//         osg::ref_ptr<Node> box = fac.createWireframeBox(robotSizeX, robotSizeY, robotSizeZ);
//         osg::PositionAttitudeTransform* trans = new osg::PositionAttitudeTransform();
//         osg::Quat q(zRot, osg::Vec3d(0, 0, 1));
//         trans->setAttitude(q);
//         trans->addChild(box);
//         p->root->addChild(trans);
//         Benching
//         
//         osg::Sphere* s1 = new osg::Sphere(osg::Vec3d(rotatedCorners.col(0)(0),
//                                                      rotatedCorners.col(0)(1),
//                                                      rotatedCorners.col(0)(2)), 0.01);
// 
//         osg::Sphere* s2 = new osg::Sphere(osg::Vec3d(rotatedCorners.col(1)(0),
//                                                      rotatedCorners.col(1)(1),
//                                                      rotatedCorners.col(1)(2)), 0.01);
//         
//         osg::Sphere* s3 = new osg::Sphere(osg::Vec3d(rotatedCorners.col(2)(0),
//                                                      rotatedCorners.col(2)(1),
//                                                      rotatedCorners.col(2)(2)), 0.01);
//         
//         osg::Sphere* s4 = new osg::Sphere(osg::Vec3d(rotatedCorners.col(3)(0),
//                                                      rotatedCorners.col(3)(1),
//                                                      rotatedCorners.col(3)(2)), 0.01);
//         
//         osg::Sphere* s5 = new osg::Sphere(osg::Vec3d(rotatedCorners.col(4)(0),
//                                                      rotatedCorners.col(4)(1),
//                                                      rotatedCorners.col(4)(2)), 0.01);
//         
//         osg::Sphere* s6 = new osg::Sphere(osg::Vec3d(rotatedCorners.col(5)(0),
//                                                      rotatedCorners.col(5)(1),
//                                                      rotatedCorners.col(5)(2)), 0.01);
//         
//         osg::Sphere* s7 = new osg::Sphere(osg::Vec3d(rotatedCorners.col(6)(0),
//                                                      rotatedCorners.col(6)(1),
//                                                      rotatedCorners.col(6)(2)), 0.01);
//         
//         osg::Sphere* s8 = new osg::Sphere(osg::Vec3d(rotatedCorners.col(7)(0),
//                                                      rotatedCorners.col(7)(1),
//                                                      rotatedCorners.col(7)(2)), 0.01);
//         
//         Geode* childGeode = new Geode();
//         childGeode->addDrawable(new osg::ShapeDrawable(s1));
//         childGeode->addDrawable(new osg::ShapeDrawable(s2));
//         childGeode->addDrawable(new osg::ShapeDrawable(s3));
//         childGeode->addDrawable(new osg::ShapeDrawable(s4));
//         childGeode->addDrawable(new osg::ShapeDrawable(s5));
//         childGeode->addDrawable(new osg::ShapeDrawable(s6));
//         childGeode->addDrawable(new osg::ShapeDrawable(s7));
//         childGeode->addDrawable(new osg::ShapeDrawable(s8));
//         p->root->addChild(childGeode);
//         
//         
//         const Eigen::Vector3d v1 = aabb.corner(Eigen::AlignedBox3d::BottomRightFloor);
//         const Eigen::Vector3d v2 = aabb.corner(Eigen::AlignedBox3d::BottomRightCeil);
//         const Eigen::Vector3d v3 = aabb.corner(Eigen::AlignedBox3d::BottomLeftFloor);
//         const Eigen::Vector3d v4 = aabb.corner(Eigen::AlignedBox3d::BottomLeftCeil);
//         const Eigen::Vector3d v5 = aabb.corner(Eigen::AlignedBox3d::TopLeftFloor);
//         const Eigen::Vector3d v6 = aabb.corner(Eigen::AlignedBox3d::TopLeftCeil);
//         const Eigen::Vector3d v7 = aabb.corner(Eigen::AlignedBox3d::TopRightFloor);
//         const Eigen::Vector3d v8 = aabb.corner(Eigen::AlignedBox3d::TopRightCeil);
//         
//         osg::Sphere* sc1 = new osg::Sphere(osg::Vec3d(v1.x(),
//                                                      v1.y(),
//                                                      v1.z()), 0.03);
//         osg::Sphere* sc2 = new osg::Sphere(osg::Vec3d(v2.x(),
//                                                      v2.y(),
//                                                      v2.z()), 0.03);
//         osg::Sphere* sc3 = new osg::Sphere(osg::Vec3d(v3.x(),
//                                                      v3.y(),
//                                                      v3.z()), 0.03);
//         osg::Sphere* sc4 = new osg::Sphere(osg::Vec3d(v4.x(),
//                                                      v4.y(),
//                                                      v4.z()), 0.03);
//         osg::Sphere* sc5 = new osg::Sphere(osg::Vec3d(v5.x(),
//                                                      v5.y(),
//                                                      v5.z()), 0.03);
//         osg::Sphere* sc6 = new osg::Sphere(osg::Vec3d(v6.x(),
//                                                      v6.y(),
//                                                      v6.z()), 0.03);
//         osg::Sphere* sc7 = new osg::Sphere(osg::Vec3d(v7.x(),
//                                                      v7.y(),
//                                                      v7.z()), 0.03);
//         osg::Sphere* sc8 = new osg::Sphere(osg::Vec3d(v8.x(),
//                                                       v8.y(),
//                                                       v8.z()), 0.03);
// 
//         childGeode->addDrawable(new osg::ShapeDrawable(sc1));
//         childGeode->addDrawable(new osg::ShapeDrawable(sc2));
//         childGeode->addDrawable(new osg::ShapeDrawable(sc3));
//         childGeode->addDrawable(new osg::ShapeDrawable(sc4));
//         childGeode->addDrawable(new osg::ShapeDrawable(sc5));
//         childGeode->addDrawable(new osg::ShapeDrawable(sc6));
//         childGeode->addDrawable(new osg::ShapeDrawable(sc7));
//         childGeode->addDrawable(new osg::ShapeDrawable(sc8));       
//         
//         
     
//         const double zRot = 0.473; 
//         osgviz::PrimitivesFactory fac(nullptr);
//         osg::ref_ptr<Node> box = fac.createWireframeBox(robotSizeX, robotSizeY, robotSizeZ);
//         osg::PositionAttitudeTransform* trans = new osg::PositionAttitudeTransform();
//         osg::Quat q(zRot, osg::Vec3d(0, 0, 1));
//         trans->setAttitude(q);
//         trans->addChild(box);
//         p->root->addChild(trans);
}

void EnvironmentXYZThetaVisualization::updateDataIntern(ugv_nav4d::EnvironmentXYZTheta const& value)
{
    p->debugCollisions = value.debugCollisions;
    p->debugRobotPositions = value.debugRobotPositions;
    p->debugRotatedCorners = value.debugRotatedBoxes;
    p->intersectionPositions = value.intersectionPositions;
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

void EnvironmentXYZThetaVisualization::setSolutionMotions(const std::vector<ugv_nav4d::Motion>& motions)
{
    p->solutionMotions = motions;
}





//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(EnvironmentXYZThetaVisualization)

