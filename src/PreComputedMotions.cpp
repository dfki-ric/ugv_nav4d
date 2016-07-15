#include "PreComputedMotions.hpp"
#include <motion_planning_libraries/sbpl/SbplMotionPrimitives.hpp>
#include <maps/grid/GridMap.hpp>
#include <dwa/SubTrajectory.hpp>

const double costScaleFactor = 1000;

RobotModel::RobotModel(double tr, double rv) : translationalVelocity(tr), rotationalVelocity(rv)
{

}

PreComputedMotions::PreComputedMotions(const motion_planning_libraries::MotionPrimitivesConfig& primitiveConfig, const RobotModel &model)
{
    motion_planning_libraries::SbplMotionPrimitives primGen(primitiveConfig);
    primGen.createPrimitives();
    
    readMotionPrimitives(primGen, model);
}


void PreComputedMotions::readMotionPrimitives(const motion_planning_libraries::SbplMotionPrimitives &primGen, const RobotModel &model)
{
    const size_t numAngles = primGen.mConfig.mNumAngles;
    const double gridResolution = primGen.mConfig.mGridSize;
    
    maps::grid::GridMap<int> dummyGrid(maps::grid::Vector2ui(10, 10), Eigen::Vector2d(gridResolution, gridResolution), 0);
    
    Eigen::Vector3d zeroGridPos(0, 0 ,0);
    dummyGrid.fromGrid(maps::grid::Index(0,0), zeroGridPos);
    
    std::cout << "Num prims is " << primGen.mListPrimitives.size() << std::endl;
    for(const motion_planning_libraries::Primitive& prim : primGen.mListPrimitives)
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
            case motion_planning_libraries::MOV_BACKWARD:
            case motion_planning_libraries::MOV_BACKWARD_TURN:
                motion.type = Motion::MOV_FORWARD;
                break;
            case motion_planning_libraries::MOV_FORWARD:
            case motion_planning_libraries::MOV_FORWARD_TURN:
                motion.type = Motion::MOV_BACKWARD;
                break;
            case motion_planning_libraries::MOV_LATERAL:
                motion.type = Motion::MOV_LATERAL;
                break;
            case motion_planning_libraries::MOV_POINTTURN:
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

    //     std::cout << "startTheta " <<  prim.mStartAngle << std::endl;
    //     std::cout << "mEndPose " <<  prim.mEndPose.transpose() << std::endl;
    // 
//         std::cout << "Adding Motion: 0, 0, " << motion.startTheta << " -> " << motion.xDiff << ", " << motion.yDiff << ", " << motion.endTheta << std::endl << std::endl;
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

