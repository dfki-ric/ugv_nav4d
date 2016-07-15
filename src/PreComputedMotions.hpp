#pragma once

#include "DiscreteTheta.hpp"
#include <limits>
#include <stdexcept>
#include <iostream>
#include <vector>
#include <base/Pose.hpp>
#include <maps/grid/Index.hpp>

namespace motion_planning_libraries
{
    class SbplMotionPrimitives;
    class MotionPrimitivesConfig;
};

class RobotModel
{
public:
    RobotModel(double tr, double rv);
    
    ///in m per sec
    double translationalVelocity;
    ///in rad per sec
    double rotationalVelocity;
};
    
struct Motion
{
    enum Type {
        MOV_FORWARD,
        MOV_BACKWARD,
        MOV_POINTTURN,
        MOV_LATERAL,
    };

    Motion(unsigned int numAngles) : endTheta(0, numAngles),startTheta(0, numAngles), baseCost(0), id(std::numeric_limits<size_t>::max()) {};
    
    int xDiff;
    int yDiff;
    DiscreteTheta endTheta;
    DiscreteTheta startTheta;
    
    double speed;
    
    Type type;
    
    /**the intermediate poses are not discrete.
        * They are relative to the starting cell*/
    std::vector<base::Pose2D> intermediatePoses;
    /**relative to starting cell */
    std::vector<maps::grid::Index> intermediateCells;
    
    int baseCost;
    
    int costMultiplier;
    
    size_t id;
    
};

class PreComputedMotions
{
    //indexed by discrete start theta
    std::vector<std::vector<Motion> > thetaToMotion;

    std::vector<Motion> idToMotion;
    
public:
    PreComputedMotions(const motion_planning_libraries::MotionPrimitivesConfig& primitiveConfig, const RobotModel &model);
    
    void readMotionPrimitives(const motion_planning_libraries::SbplMotionPrimitives& primGen, const RobotModel& model);
    
    void setMotionForTheta(const Motion &motion, const DiscreteTheta &theta);
    
    void preComputeCost(Motion &motion, const RobotModel &model);
    
    const std::vector<Motion> &getMotionForStartTheta(const DiscreteTheta &theta) const
    {
        if(theta.getTheta() >= (int)thetaToMotion.size())
        {
            std::cout << "Input theta is " << theta.getTheta();
            throw std::runtime_error("Internal error, motion for requested theta ist not available");
        }
        return thetaToMotion.at(theta.getTheta());
    };
    
    const Motion &getMotion(std::size_t id) const;    
};
