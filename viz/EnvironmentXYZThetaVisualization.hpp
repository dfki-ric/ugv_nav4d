#ifndef ugv_nav4d_EnvironmentXYZThetaVisualization_H
#define ugv_nav4d_EnvironmentXYZThetaVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <osg/Geometry>
#include <ugv_nav4d/EnvironmentXYZTheta.hpp>
#include <ugv_nav4d/PreComputedMotions.hpp>

namespace vizkit3d
{
    class EnvironmentXYZThetaVisualization
        : public vizkit3d::Vizkit3DPlugin<ugv_nav4d::EnvironmentXYZTheta>
        , boost::noncopyable
    {
    Q_OBJECT
    
    Q_PROPERTY(int numSuccs READ getNumSuccs WRITE setNumSuccs)
    Q_PROPERTY(bool showHeuristic READ getShowHeuristic WRITE setShowHeuristic)
    
    public:
        EnvironmentXYZThetaVisualization();
        ~EnvironmentXYZThetaVisualization();

    Q_INVOKABLE void updateData(ugv_nav4d::EnvironmentXYZTheta const &sample)
    {vizkit3d::Vizkit3DPlugin<ugv_nav4d::EnvironmentXYZTheta>::updateData(sample);}
    
    public slots:
      void setGridSize(const double gridSize); //size of one grid cell
      void setStartPos(const double x, const double y, const double z);
      void setGoalPos(const double x, const double y, const double z);
      void setHeuristic(const std::vector<Eigen::Vector4d>& cost);
      void setCollisionPoses(std::vector<base::Pose>& poses);
      void setRobotHalfSize(const Eigen::Vector3d& value);
      void setSuccessors(std::vector<Eigen::Vector3d>& succs);
      
      int getNumSuccs();
      void setNumSuccs(int val);
      
      void setShowHeuristic(bool val);
      bool getShowHeuristic();

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(ugv_nav4d::EnvironmentXYZTheta const& plan);
        
    private:
        struct Data;
        int numSuccs = 999999;
        bool showHeuristic = false;
        Data* p;
    };
}
#endif
