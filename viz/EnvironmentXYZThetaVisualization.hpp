#ifndef ugv_nav4d_EnvironmentXYZThetaVisualization_H
#define ugv_nav4d_EnvironmentXYZThetaVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <ugv_nav4d/EnvironmentXYZTheta.hpp>

namespace vizkit3d
{
    class EnvironmentXYZThetaVisualization
        : public vizkit3d::Vizkit3DPlugin<EnvironmentXYZTheta>
        , boost::noncopyable
    {
    Q_OBJECT
    public:
        EnvironmentXYZThetaVisualization();
        ~EnvironmentXYZThetaVisualization();

    Q_INVOKABLE void updateData(EnvironmentXYZTheta const &sample)
    {vizkit3d::Vizkit3DPlugin<EnvironmentXYZTheta>::updateData(sample);}

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(EnvironmentXYZTheta const& plan);
        
    private:
        struct Data;
        Data* p;
    };
}
#endif
