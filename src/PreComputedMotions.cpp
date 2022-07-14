#include "PreComputedMotions.hpp"
#include <maps/grid/GridMap.hpp>
#include <cmath>
#include <base/Angle.hpp>

namespace ugv_nav4d
{
using namespace sbpl_spline_primitives;


PreComputedMotions::PreComputedMotions(const SplinePrimitivesConfig& primitiveConfig,
                                       const Mobility& mobilityConfig):
    primitives(primitiveConfig),
    mobilityConfig(mobilityConfig)
{
}


void PreComputedMotions::computeMotions(double obstGridResolution, double travGridResolution)
{
    if(fabs(primitives.getConfig().gridSize - travGridResolution) > 1E-5)
    {
        std::cout << "PreComputedMotions::computeMotions: Error grid size and trav size do not match" << std::endl;
        throw std::runtime_error("PreComputedMotions::computeMotions: Error grid size and trav size do not match");
    }
    
    readMotionPrimitives(primitives, mobilityConfig, obstGridResolution, travGridResolution);
}

void PreComputedMotions::sampleOnResolution(double gridResolution,base::geometry::Spline2 spline, std::vector<PoseWithCell> &result, std::vector<CellWithPoses> &fullResult)
{
    maps::grid::GridMap<int> dummyGrid(maps::grid::Vector2ui(10, 10), base::Vector2d(gridResolution, gridResolution), 0);

    CellWithPoses curPoses;
    curPoses.cell = maps::grid::Index(0,0);
    
    //sample spline:
    const double stepDist = gridResolution / 16.0;
    std::vector<double> parameters;
    //NOTE we dont need the points, but there is no sample() api that returns parameters only
    const std::vector<base::geometry::Spline2::vector_t> points = spline.sample(stepDist, &parameters);
    assert(parameters.size() == points.size());
    for(size_t i = 0; i < parameters.size(); ++i)
    {
        const double param = parameters[i];
        base::Vector2d point, tangent;
        std::tie(point,tangent) = spline.getPointAndTangent(param);
        const base::Orientation2D orientation(std::atan2(tangent.y(), tangent.x()));


        const base::Pose2D pose(point, orientation);
        
        //point needs to be offset to the middle of the grid, 
        //as all path computation also starts in the middle
        //if not we would get a wrong diff
        point += base::Vector2d(gridResolution /2.0, gridResolution /2.0);

        maps::grid::Index diff;
        if(!dummyGrid.toGrid(base::Vector3d(point.x(), point.y(), 0), diff, false))
            throw std::runtime_error("Internal Error : Cannot convert intermediate Pose to grid cell");
        
        if(curPoses.cell != diff)
        {
            fullResult.push_back(curPoses);
            
            //Find best match for collision check
            PoseWithCell pwc;
            pwc.cell = curPoses.cell;
            pwc.pose = getPointClosestToCellMiddle(curPoses, gridResolution);
            result.push_back(pwc);

            curPoses.poses.clear();
            curPoses.cell = diff;
        }
        
        curPoses.poses.push_back(pose);
    }
    
    fullResult.push_back(curPoses);
    
    //Find best match for collision check
    PoseWithCell pwc;
    pwc.cell = curPoses.cell;
    pwc.pose = getPointClosestToCellMiddle(curPoses, gridResolution);
    result.push_back(pwc);

    assert(result.size() > 0); //at least the end pose should always be part of the steps

}

void PreComputedMotions::readMotionPrimitives(const SbplSplineMotionPrimitives& primGen,
                                              const Mobility& mobilityConfig,
                                              double obstGridResolution, double travGridResolution)
{
    const int numAngles = primGen.getConfig().numAngles;
    const double maxCurvature = calculateCurvatureFromRadius(mobilityConfig.minTurningRadius);

    for(int angle = 0; angle < numAngles; ++angle)
    {
        for(const SplinePrimitive& prim : primGen.getPrimitiveForAngle(angle))
        {
            //NOTE the const cast is only here because for some reason getCurvatureMax() is non-const (but shouldnt be)
            if(prim.motionType != SplinePrimitive::SPLINE_POINT_TURN && //cannot call getCurvatureMax on point turns cause spl ne is not initalized
               const_cast<SplinePrimitive&>(prim).spline.getCurvatureMax() > maxCurvature)
               {
                   continue;
               }   
         
            Motion motion(numAngles);

            motion.xDiff = prim.endPosition[0];
            motion.yDiff = prim.endPosition[1];
            motion.endTheta =  DiscreteTheta(static_cast<int>(prim.endAngle), numAngles);
            motion.startTheta = DiscreteTheta(static_cast<int>(prim.startAngle), numAngles);
            motion.costMultiplier = 1; //is changed in the switch-case below
           
            switch(prim.motionType)
            {
                case SplinePrimitive::SPLINE_MOVE_FORWARD:
                    motion.type = Motion::Type::MOV_FORWARD;
                    motion.costMultiplier = mobilityConfig.multiplierForwardTurn;
                    break;
                case SplinePrimitive::SPLINE_MOVE_BACKWARD:
                    motion.type = Motion::Type::MOV_BACKWARD;
                    motion.costMultiplier = mobilityConfig.multiplierBackwardTurn;
                    break;
                case SplinePrimitive::SPLINE_MOVE_LATERAL:
                    motion.type = Motion::Type::MOV_LATERAL;
                    motion.costMultiplier = mobilityConfig.multiplierLateral;
                    break;
                case SplinePrimitive::SPLINE_POINT_TURN:
                    motion.type = Motion::Type::MOV_POINTTURN;
                    motion.costMultiplier = mobilityConfig.multiplierPointTurn;
                    break;
                default:
                    throw std::runtime_error("Got Unsupported movement");
            }
            
            //there are no intermediate steps for point turns
            if(prim.motionType != SplinePrimitive::SPLINE_POINT_TURN)
            {
                sampleOnResolution(travGridResolution, prim.spline, motion.intermediateStepsTravMap, motion.fullSplineSamples);
                std::vector<CellWithPoses> dummy;
                sampleOnResolution(obstGridResolution, prim.spline, motion.intermediateStepsObstMap, dummy);
            }
            computeSplinePrimCost(prim, mobilityConfig, motion);
            
            if (motion.translationlDist > mobilityConfig.maxMotionCurveLength) //1.3 is slower but trajectories are curvy , 1.0 is faster with more linear trajectories
            continue;
            
            //orientations for backward motions need to be inverted
            if(motion.type == Motion::Type::MOV_BACKWARD)
            {
                for(PoseWithCell& pwc : motion.intermediateStepsTravMap)
                    pwc.pose.orientation = base::Angle::fromRad(pwc.pose.orientation).flipped().getRad();
                for(PoseWithCell& pwc : motion.intermediateStepsObstMap)
                    pwc.pose.orientation = base::Angle::fromRad(pwc.pose.orientation).flipped().getRad();
            }

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
        if(m.xDiff == motion.xDiff && m.yDiff == motion.yDiff &&
           m.endTheta == motion.endTheta && m.type == motion.type)
        {
            std::string type;
            switch(m.type)
            {
                case Motion::Type::MOV_FORWARD:  type ="MOV_FORWARD" ; break;
                case Motion::Type::MOV_BACKWARD: type ="MOV_BACKWARD" ; break;
                case Motion::Type::MOV_POINTTURN:type ="MOV_POINTTURN" ; break;
                case Motion::Type::MOV_LATERAL:  type ="MOV_LATERAL" ; break;
                default:
                    throw std::runtime_error("ERROR: motion without valid type: ");
                    
            }
            std::cout << "WARNING: motion already exists (skipping): " <<  m.xDiff << ", " << m.yDiff << ", " << m.endTheta << type << std::endl;
            //TODO add check if intermediate poses are similar
            return;
        }
    }
    
    Motion copy = motion;
    copy.id = idToMotion.size();

    idToMotion.push_back(copy);
    thetaToMotion[theta.getTheta()].push_back(copy);
}

base::Pose2D PreComputedMotions::getPointClosestToCellMiddle(const CellWithPoses& cwp, const double gridResolution)
{
    //dummyGrid is used to convert between grid indices and positions
    maps::grid::GridMap<int> dummyGrid(maps::grid::Vector2ui(10, 10), base::Vector2d(gridResolution, gridResolution), 0);
    
    maps::grid::Vector3d cellCenter;
    if(!dummyGrid.fromGrid(cwp.cell, cellCenter, 0, false))
        throw std::runtime_error("Internal Error : Cannot convert index to cell position");
    
    const maps::grid::Vector2d cellCenter2D = cellCenter.topRows(2);
    double closestDist = std::numeric_limits<double>::max();
    assert(cwp.poses.size() > 0);
    base::Pose2D closestPose = cwp.poses.front();
    for(const base::Pose2D& pose : cwp.poses)
    {
        const Eigen::Vector2d centeredPos = pose.position + base::Vector2d(gridResolution /2.0, gridResolution /2.0);
        const double dist = (cellCenter2D - centeredPos).norm();
        if(dist < closestDist)
        {
            closestDist = dist;
            closestPose = pose;
        }
        
        maps::grid::Index testIdx;
        dummyGrid.toGrid(Eigen::Vector3d(centeredPos.x(), centeredPos.y(), 0), testIdx, false);
        
        assert(testIdx == cwp.cell);
    }
    
    return closestPose;
}


void PreComputedMotions::computeSplinePrimCost(const SplinePrimitive& prim,
                                               const Mobility& mobilityConfig,
                                               Motion& outMotion) const
{
    
    double linearDist = 0;
    double angularDist = 0;
    if(prim.motionType == SplinePrimitive::SPLINE_POINT_TURN)
    {
        angularDist = outMotion.startTheta.shortestDist(outMotion.endTheta).getRadian();
    }
    else
    {
        linearDist = prim.spline.getCurveLength();;
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
    }
    
    outMotion.baseCost = Motion::calculateCost(linearDist, angularDist, mobilityConfig.translationSpeed,
                                               mobilityConfig.rotationSpeed, outMotion.costMultiplier);
    assert(outMotion.baseCost >= 0);
    outMotion.translationlDist = linearDist;
    outMotion.angularDist = angularDist;
}

int Motion::calculateCost(double translationalDist, double angularDist, double translationVelocity,
                          double angularVelocity, double costMultiplier)
{
    const double translationTime = translationalDist / translationVelocity;
    const double angularTime = angularDist / angularVelocity;
    
    //use ulonglong to catch overflows caused by large cost multipliers
    unsigned long long cost = ceil(std::max(angularTime, translationTime) * Motion::costScaleFactor * costMultiplier);
    
    if(cost > std::numeric_limits<int>::max())
    {
        std::cerr << "WARNING: primitive cost too large for int. Clipping to int_max." << std::endl;
        return std::numeric_limits<int>::max();
    }
    else
        return cost;    
}

const Motion& PreComputedMotions::getMotion(std::size_t id) const
{
    return idToMotion.at(id);
}

const SbplSplineMotionPrimitives& PreComputedMotions::getPrimitives() const
{
    return primitives;
}

double PreComputedMotions::calculateCurvatureFromRadius(const double r)
{
    assert(r > 0);
    /*
        Curvature:
        c = abs(f''(x)/(1 + f'(x)^2)^(3/2))
        
        Circle equation:
        f(x)   = sqrt(r^2 - x^2)       
        f'(x)  = -x/sqrt(r^2-x^2)
        f''(x) = -r^2/((r^2-x^2)^(3/2))
        
        Since the curvature of a circle is constant the value of x doesnt matter.
        x has to be smaller than r since we calc sqrt(r^2 - x ^2) which is only defined for positive values.
     */
    const double x = r/2.0;
    const double x2 = x * x;
    const double r2 = r * r;
    const double df = - (x / std::sqrt(r2 - x2));
    const double ddf = - (r2 / std::pow(r2 - x2, 3.0 / 2.0));
    assert(!std::isnan(df) && !std::isinf(df));
    assert(!std::isnan(ddf) && !std::isinf(ddf));
    
    const double df2 = df * df;
    const double c = (std::abs(ddf) / std::pow(1 + df2, 3.0 / 2.0));
    
    return c;
}

const std::vector< Motion >& PreComputedMotions::getMotionForStartTheta(const DiscreteTheta& theta) const
{
    if(theta.getTheta() >= (int)thetaToMotion.size())
    {
        throw std::runtime_error("Internal error, motion for requested theta ist not available. Input  theta:" + std::to_string(theta.getTheta()));
    }
    return thetaToMotion.at(theta.getTheta());
}


double Motion::costScaleFactor = 1000.0; //shift by 3 decimals to keep the decimals as int

}
