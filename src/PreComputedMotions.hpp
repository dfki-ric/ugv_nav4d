#pragma once

#include "DiscreteTheta.hpp"
#include <limits>
#include <stdexcept>
#include <iostream>
#include <vector>
#include <base/Pose.hpp>
#include <maps/grid/Index.hpp>
#include <motion_planning_libraries/sbpl/SbplMotionPrimitives.hpp>
#include <motion_planning_libraries/sbpl/SbplSplineMotionPrimitives.hpp>
#include <base/Trajectory.hpp>



namespace motion_planning_libraries
{
    class MotionPrimitivesConfig;
};

namespace ugv_nav4d
{

struct PoseWithCell
{
    base::Pose2D pose;
    maps::grid::Index cell;
};

class Motion
{
    /**used to scale the costs because costs
     * are int but real costs are most likely small doubles*/
    static double costScaleFactor;
    
public:
    enum Type {
        MOV_FORWARD,
        MOV_BACKWARD,
        MOV_POINTTURN,
        MOV_LATERAL,
    };

    Motion(unsigned int numAngles = 0) : endTheta(0, numAngles),startTheta(0, numAngles), baseCost(0), id(std::numeric_limits<size_t>::max()) {};
    
    static int calculateCost(double translationalDist, double angularDist, double translationVelocity, double angularVelocity, double costMultiplier);
    
    int xDiff;
    int yDiff;
    DiscreteTheta endTheta;
    DiscreteTheta startTheta;
    
    double speed; //FIXME is this used?
    
    Type type;
    
    /**the intermediate poses are not discrete.
     * They are relative to the starting cell*/
    std::vector<PoseWithCell> intermediateSteps;
    
    int baseCost; //time the robot needs to follow the primivite scaled by some factors
    int costMultiplier;//is used to scale the baseCost
    double translationlDist; //translational length of the motion
    double angularDist; //angular length of the motion
    
    size_t id;
    
};

class PreComputedMotions
{
    //indexed by discrete start theta
    std::vector<std::vector<Motion> > thetaToMotion;
    std::vector<Motion> idToMotion;
    motion_planning_libraries::SbplMotionPrimitives primitives;
         
public:
    /**Initialize using spline based primitives.
     * @param mobilityConfig Will be used to configure and filter the splines.
     *                       mobilityConfig.mMinTurningRadius will be used to
     *                       filter the splines and remove all splines with a smaller turning radius.*/
    PreComputedMotions(const motion_planning_libraries::SplinePrimitivesConfig& primitiveConfig,
                       const motion_planning_libraries::Mobility& mobilityConfig);
    
    void readMotionPrimitives(const motion_planning_libraries::SbplSplineMotionPrimitives& primGen,
                              const motion_planning_libraries::Mobility& mobilityConfig);
    
    void setMotionForTheta(const Motion &motion, const DiscreteTheta &theta);
    
    void preComputeCost(Motion &motion);
    
    const std::vector<Motion> &getMotionForStartTheta(const DiscreteTheta &theta) const;
    
    const Motion &getMotion(std::size_t id) const; 
    
    const motion_planning_libraries::SbplMotionPrimitives& getPrimitives() const;
    
    /**Calculate the curvature of a circle based on the radius of the circle */
    static double calculateCurvatureFromRadius(const double r);
private:
    
    void computeSplinePrimCost(const motion_planning_libraries::SplinePrimitive& prim,
                               const motion_planning_libraries::Mobility& mobilityConfig, Motion& outMotion) const;
    
};


}