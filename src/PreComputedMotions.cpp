#include "PreComputedMotions.hpp"
#include <maps/grid/GridMap.hpp>
#include <dwa/SubTrajectory.hpp>

namespace ugv_nav4d
{
using namespace motion_planning_libraries;



RobotModel::RobotModel(double tr, double rv) : translationalVelocity(tr), rotationalVelocity(rv)
{

}

PreComputedMotions::PreComputedMotions(const SplinePrimitivesConfig& primitiveConfig,
                                       const RobotModel& model,
                                       const motion_planning_libraries::Mobility& mobilityConfig)
{
    SbplSplineMotionPrimitives prims(primitiveConfig);
    readMotionPrimitives(prims, model, mobilityConfig);
}


void PreComputedMotions::readMotionPrimitives(const SbplSplineMotionPrimitives& primGen,
                                              const RobotModel& model,
                                              const motion_planning_libraries::Mobility& mobilityConfig)
{
    const int numAngles = primGen.getConfig().numAngles;
    const double gridResolution = primGen.getConfig().gridSize;
    
    maps::grid::GridMap<int> dummyGrid(maps::grid::Vector2ui(10, 10), base::Vector2d(gridResolution, gridResolution), 0);

    for(int angle = 0; angle < numAngles; ++angle)
    {
        for(const SplinePrimitive& prim : primGen.getPrimitiveForAngle(angle))
        {
            Motion motion(numAngles);

            motion.xDiff = prim.endPosition[0];
            motion.yDiff = prim.endPosition[1];
            motion.endTheta =  DiscreteTheta(static_cast<int>(prim.endAngle), numAngles);
            motion.startTheta = DiscreteTheta(static_cast<int>(prim.startAngle), numAngles);
            motion.costMultiplier = 1; //is changed in the switch-case below
            motion.speed = mobilityConfig.mSpeed;
           
            switch(prim.motionType)
            {
                case SplinePrimitive::SPLINE_MOVE_FORWARD:
                    motion.type = Motion::MOV_FORWARD;
                    motion.costMultiplier = mobilityConfig.mMultiplierForwardTurn;
                    break;
                case SplinePrimitive::SPLINE_MOVE_BACKWARD:
                    motion.type = Motion::MOV_BACKWARD;
                    motion.costMultiplier = mobilityConfig.mMultiplierBackwardTurn;
                    break;
                case SplinePrimitive::SPLINE_MOVE_LATERAL:
                    motion.type = Motion::MOV_LATERAL;
                    motion.costMultiplier = mobilityConfig.mMultiplierLateral;
                    break;
                default:
                    throw std::runtime_error("Got Unsupported movement");
            }
            
            const double stepDist = gridResolution / 4.0;
            std::vector<double> parameters;
            //NOTE we dont need the points, but there is no sample() api that returns parameters only
            const std::vector<base::geometry::Spline2::vector_t> points = prim.spline.sample(stepDist, &parameters);
            assert(parameters.size() == points.size());
            
            maps::grid::Index lastIdx(0,0);
            
            for(size_t i = 0; i < parameters.size(); ++i)
            {
                const double param = parameters[i];
                base::Vector2d point, tangent;
                std::tie(point,tangent) = prim.spline.getPointAndTangent(param);
                const base::Orientation2D orientation(std::atan2(tangent.y(), tangent.x()));
                const base::Pose2D pose(point, orientation);
                maps::grid::Index diff;
                if(!dummyGrid.toGrid(base::Vector3d(point.x(), point.y(), 0), diff, false))
                    throw std::runtime_error("Internal Error : Cannot convert intermediate Pose to grid cell");
                if(lastIdx != diff ||
                   i == parameters.size() - 1) //make sure that the last position is added, even if there already is a pose in this cell
                {
                    PoseWithCell s;
                    s.cell = diff;
                    s.pose = pose;
                    motion.intermediateSteps.push_back(s);
                    if((lastIdx - diff).norm() > 1)
                        throw std::runtime_error("skipped a cell");
                    
                    lastIdx = diff;
                } 
            }
            assert(motion.intermediateSteps.size() > 0); //at least the end pose should always be part of the steps
            computeSplinePrimCost(prim, model, motion);
            setMotionForTheta(motion, motion.startTheta);
        }
    }
}


void PreComputedMotions::setMotionForTheta(const Motion& motion, const DiscreteTheta& theta)
{
    if((int)thetaToMotion.size() <= theta.getTheta())
    {
        thetaToMotion.resize(theta.getTheta() + 1);
    }
    
    
    //check if a motion to this target destination already exist, if yes skip it.
    for(const Motion& m : thetaToMotion[theta.getTheta()])
    {
        if(m.xDiff == motion.xDiff && m.yDiff == motion.yDiff && m.endTheta == motion.endTheta)
        {
            std::cout << "WARNING: motion already exists (skipping): " << m.xDiff << ", " << m.yDiff << ", " << m.endTheta << std::endl;
            //TODO add check if intermediate poses are similar
            return;
        }
    }
    
    Motion copy = motion;
    copy.id = idToMotion.size();

    idToMotion.push_back(copy);
    thetaToMotion[theta.getTheta()].push_back(copy);
}

void PreComputedMotions::computeSplinePrimCost(const SplinePrimitive& prim,
                                              const RobotModel &model,
                                              Motion& outMotion) const
{
    double linearDist = prim.spline.getCurveLength();
    assert(linearDist > 0);
    double angularDist = 0;
    const double stepDist = prim.spline.getGeometricResolution();
    std::vector<double> parameters;
    
    const std::vector<base::geometry::Spline2::vector_t> points = prim.spline.sample(stepDist, &parameters);
    assert(parameters.size() == points.size());
    
    for(int i = 0; i < ((int)points.size()) - 1; ++i)
    {
        const double dist = prim.spline.getCurveLength(parameters[i], parameters[i+1], 0.01);
        const double curvature = prim.spline.getCurvature(parameters[i]); //assume that the curvature is const between i and i+1
        angularDist += dist / linearDist  * std::abs(curvature);
    }
    assert(angularDist >= 0);
    
    const double translationalVelocity = std::min(model.translationalVelocity, outMotion.speed);
    const double rotationalVelocity = model.rotationalVelocity;
    const double linearTime = linearDist / translationalVelocity;
    const double angularTime = angularDist / rotationalVelocity;
    
    //use ulonglong to catch overflows caused by large cost multipliers
    unsigned long long cost = ceil(std::max(angularTime, linearTime) * costScaleFactor * outMotion.costMultiplier);
    
    if(cost > std::numeric_limits<int>::max())
    {
        std::cerr << "WARNING: primitive cost too large for int. Clipping to int_max." << std::endl;
        outMotion.baseCost = std::numeric_limits<int>::max();
    }
    else
        outMotion.baseCost = cost;
    
    assert(outMotion.baseCost >= 0);
}

const Motion& PreComputedMotions::getMotion(std::size_t id) const
{
    return idToMotion.at(id);
}

const SbplMotionPrimitives& PreComputedMotions::getPrimitives() const
{
    return primitives;
}


}