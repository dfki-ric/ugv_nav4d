#pragma once

#include "DiscreteTheta.hpp"
#include "Config.hpp"
#include <limits>
#include <stdexcept>
#include <iostream>
#include <vector>
#include <base/Pose.hpp>
#include <maps/grid/Index.hpp>
#include <sbpl_spline_primitives/SbplSplineMotionPrimitives.hpp>
#include <base/Trajectory.hpp>



namespace sbpl_spline_primitives
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

struct CellWithPoses 
{
    maps::grid::Index cell;
    std::vector<base::Pose2D> poses;
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
     * They are relative to the starting cell.
     * The Poses start from (0/0), while the 
     * cell idx is computed from the center of the start cell + pose
    */
    std::vector<PoseWithCell> intermediateStepsTravMap;

    /**the intermediate poses are not discrete.
     * They are relative to the starting cell.
     * The Poses start from (0/0), while the 
     * cell idx is computed from the center of the start cell + pose
    */
    std::vector<PoseWithCell> intermediateStepsObstMap;
    
    /**
     * This vector contains a full resolution
     * sample of the motion primitive, together
     * with the cell the poses are supposed to 
     * be in. Poses are relative to (0/0), wile
     * the cellIndex is computed relative to the 
     * center of the start cell.
     * */
    std::vector<CellWithPoses> fullSplineSamples;
    
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
    sbpl_spline_primitives::SbplSplineMotionPrimitives primitives;
    Mobility mobilityConfig;
public:
    /**Initialize using spline based primitives.
     * @param mobilityConfig Will be used to configure and filter the splines.
     *                       mobilityConfig.mMinTurningRadius will be used to
     *                       filter the splines and remove all splines with a smaller turning radius.*/
    PreComputedMotions(const sbpl_spline_primitives::SplinePrimitivesConfig& primitiveConfig,
                       const Mobility& mobilityConfig);
    
    void readMotionPrimitives(const sbpl_spline_primitives::SbplSplineMotionPrimitives& primGen,
                              const Mobility& mobilityConfig,
                              double obstGridResolution, double travGridResolution);
    
    void computeMotions(double obstGridResolution, double travGridResolution);
    
    void setMotionForTheta(const Motion &motion, const DiscreteTheta &theta);
    
    void preComputeCost(Motion &motion);
    
    const std::vector<Motion> &getMotionForStartTheta(const DiscreteTheta &theta) const;
    
    const Motion &getMotion(std::size_t id) const; 
    
    const sbpl_spline_primitives::SbplSplineMotionPrimitives& getPrimitives() const;
    
    /**Calculate the curvature of a circle based on the radius of the circle */
    static double calculateCurvatureFromRadius(const double r);
private:
    
    void sampleOnResolution(double gridResolution, base::geometry::Spline2 spline, std::vector< ugv_nav4d::PoseWithCell >& result, std::vector< ugv_nav4d::CellWithPoses >& fullResult);
    
    base::Pose2D getPointClosestToCellMiddle(const ugv_nav4d::CellWithPoses& cwp, const double gridResolution);
    
    
    void computeSplinePrimCost(const sbpl_spline_primitives::SplinePrimitive& prim,
                               const Mobility& mobilityConfig, Motion& outMotion) const;
    
};


}
