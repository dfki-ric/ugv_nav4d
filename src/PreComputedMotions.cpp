#include "PreComputedMotions.hpp"
#include <maps/grid/GridMap.hpp>
#include <dwa/SubTrajectory.hpp>

namespace ugv_nav4d
{
using namespace motion_planning_libraries;
const double costScaleFactor = 1000;

RobotModel::RobotModel(double tr, double rv) : translationalVelocity(tr), rotationalVelocity(rv)
{

}

PreComputedMotions::PreComputedMotions(const MotionPrimitivesConfig& primitiveConfig,
                                       const RobotModel &model) : primitives(primitiveConfig)
{
    primitives.createPrimitives();
    readMotionPrimitives(primitives, model);
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

void PreComputedMotions::readMotionPrimitives(const SbplMotionPrimitives &primGen, const RobotModel &model)
{
    const size_t numAngles = primGen.mConfig.mNumAngles;
    const double gridResolution = primGen.mConfig.mGridSize;
    
    maps::grid::GridMap<int> dummyGrid(maps::grid::Vector2ui(10, 10), Eigen::Vector2d(gridResolution, gridResolution), 0);
    
    Eigen::Vector3d zeroGridPos(0, 0 ,0);
    dummyGrid.fromGrid(maps::grid::Index(0,0), zeroGridPos);
    
    for(const Primitive& prim : primGen.mListPrimitives)
    {
        Motion motion(numAngles);

        motion.xDiff = prim.mEndPose[0];
        motion.yDiff = prim.mEndPose[1];
        motion.endTheta =  DiscreteTheta(static_cast<int>(prim.mEndPose[2]), numAngles);
        motion.startTheta = DiscreteTheta(static_cast<int>(prim.mStartAngle), numAngles);
        motion.costMultiplier = prim.mCostMultiplier;

        motion.speed = prim.mSpeed;
        
        switch(prim.mMovType)
        {
            case MOV_BACKWARD:
            case MOV_BACKWARD_TURN:
                motion.type = Motion::MOV_FORWARD;
                break;
            case MOV_FORWARD:
            case MOV_FORWARD_TURN:
                motion.type = Motion::MOV_BACKWARD;
                break;
            case MOV_LATERAL:
                motion.type = Motion::MOV_LATERAL;
                break;
            case MOV_POINTTURN:
                motion.type = Motion::MOV_POINTTURN;
                break;
            default:
                throw std::runtime_error("Got Unsupported movement");
        }
        
        std::vector<base::Pose2D> poses;
        bool allPositionsSame = true;
        base::Vector2d firstPos = prim.mIntermediatePoses.front().topRows(2);
        for(const base::Vector3d& pose : prim.mIntermediatePoses)
        {
            poses.emplace_back(base::Position2D(pose[0], pose[1]), pose[2]);
            if(pose.topRows(2) != firstPos)
            {
                allPositionsSame = false;
            }
        }

        if(!allPositionsSame)
        {
            //fit a spline through the primitive's intermediate points and use that spline to
            //create intermediate points for the motion.
            //This is done to ensure that every cell is hit by one intermediate point.
            //if the positions are different, normal spline interpolation works
            SubTrajectory spline;
            spline.interpolate(poses);
            const double splineLength = spline.getDistToGoal(spline.getStartParam());
            //set stepDist so small that we are guaranteed to oversample
            const double stepDist = gridResolution / 4.0;
            double currentDist = 0;
            double currentParam = spline.getStartParam();
            maps::grid::Index lastIdx(0,0);
            while(currentDist < splineLength)
            {
                currentDist += stepDist;
                currentParam = spline.advance(currentParam, stepDist);
                const base::Pose2D currentPose = spline.getIntermediatePointNormalized(currentParam);
                
                //convert pose to grid
                base::Vector3d position(currentPose.position.x(), currentPose.position.y(), 0);
                //everything is centered in the grid cells, therefor we need to add the zero pos here,
                //to get the intermediate cells in the grid
                position += zeroGridPos;
                maps::grid::Index diff;
                //only add pose if it is in a different cell than the one before
                if(!dummyGrid.toGrid(position, diff, false))
                {
                    throw std::runtime_error("Internal Error : Cannot convert intermediate Pose to grid cell");
                }
                
                if(lastIdx != diff)
                {
                    PoseWithCell s;
                    s.cell = diff;
                    s.pose = currentPose;
                    motion.intermediateSteps.push_back(s);
    //               std::cout << "intermediate poses: " << currentPose.position.transpose() << ", " << currentPose.orientation << 
    //                            "[" << diff.transpose() << "]" << std::endl;
                    if((lastIdx - diff).norm() > 1)
                        throw std::runtime_error("Yea");
                    
                    lastIdx = diff;
                }            
            }
        }
        else
        {
            //add rotation intermediate poses for collision checking
            for(const auto &p: poses)
            {
                PoseWithCell s;
                s.pose = p;
                s.cell = maps::grid::Index(0,0);
                motion.intermediateSteps.push_back(s);
            }
        }

        preComputeCost(motion, model);

        setMotionForTheta(motion, motion.startTheta);
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


void PreComputedMotions::preComputeCost(Motion& motion, const RobotModel &model)
{
    //compute linear and angular time
    double linear_distance = 0;
    Eigen::Vector2d lastPos(0, 0);
    for(const PoseWithCell &pwc: motion.intermediateSteps)
    {
        linear_distance += (lastPos - pwc.pose.position).norm();
        lastPos = pwc.pose.position;
    }

    double translationalVelocity = model.translationalVelocity;
        translationalVelocity = std::min(model.translationalVelocity, motion.speed);
    
    double linear_time = linear_distance / translationalVelocity;

    double angular_distance = motion.endTheta.shortestDist(motion.startTheta).getRadian();
    
    double angular_time = angular_distance / model.rotationalVelocity;
    
    motion.baseCost = ceil(std::max(angular_time, linear_time) * costScaleFactor * motion.costMultiplier);

//     std::cout << "Motion " << motion.xDiff << " " << motion.yDiff << " " << motion.endTheta << 
//     " lin dist " <<  linear_distance << " time " << linear_time << " angle diff " 
//     << angular_distance << " time " << angular_time << " cost " << motion.baseCost << " multiplier " << motion.costMultiplier <<  std::endl;
    

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