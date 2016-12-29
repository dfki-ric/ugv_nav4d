#ifndef ugv_nav4d_EnvironmentXYZThetaVisualization_H
#define ugv_nav4d_EnvironmentXYZThetaVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <osg/Geometry>
#include <ugv_nav4d/EnvironmentXYZTheta.hpp>
#include <ugv_nav4d/PreComputedMotions.hpp>

using namespace ugv_nav4d;

namespace vizkit3d
{
    class EnvironmentXYZThetaVisualization
        : public vizkit3d::Vizkit3DPlugin<ugv_nav4d::EnvironmentXYZTheta>
        , boost::noncopyable
    {
    Q_OBJECT
    
    Q_PROPERTY(int numSuccs READ getNumSuccs WRITE setNumSuccs)
    Q_PROPERTY(bool showHeuristic READ getShowHeuristic WRITE setShowHeuristic)
    Q_PROPERTY(bool showSlopes READ getShowSlopes WRITE setShowSlopes)
    Q_PROPERTY(bool showAllowedSlopes READ getshowAllowedSlopes WRITE setshowAllowedSlopes)
    Q_PROPERTY(bool showCollisions READ getShowCollisions WRITE setShowCollisions)
    
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
      void setSlopes(const std::vector<Eigen::Vector4d>& slopes);
      void setSlopeDirs(const std::vector<Eigen::Matrix<double, 2, 3>>& slopeDirs);
      
      void setSlopeDebug(const std::vector<EnvironmentXYZTheta::DebugSlopeData>& data);
      void setSlopeDebugCandidate(const std::vector<EnvironmentXYZTheta::DebugSlopeCandidate>& data);
      
      int getNumSuccs();
      void setNumSuccs(int val);
      
      void setShowHeuristic(bool val);
      bool getShowHeuristic();
      
      bool getShowCollisions();
      void setShowCollisions(bool val);
      
      bool getShowSlopes();
      void setShowSlopes(bool val);
      
      bool getshowAllowedSlopes();
      void setshowAllowedSlopes(bool val);

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(ugv_nav4d::EnvironmentXYZTheta const& plan);
        
    private:
        struct Data;
        int numSuccs = 999999;
        bool showHeuristic = false;
        bool showCollisions = false;
        bool showSlopes = false;
        bool showAllowedSlopes = true;
        Data* p;
    };
}
#endif
