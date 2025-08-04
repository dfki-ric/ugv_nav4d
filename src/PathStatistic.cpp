#include "PathStatistic.hpp"
#include <unordered_set>
#include <deque>

#ifdef ENABLE_DEBUG_DRAWINGS
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>
#include <vizkit3d_debug_drawings/DebugDrawingColors.hpp>
#endif

ugv_nav4d::PathStatistic::Stats::Stats() :
    obstacles(0), 
    frontiers(0)
{
    //check that all patch types (that we use in here) are valid array indices
    //otherwise we might access the array out of bounds
    static_assert(maps::grid::TraversabilityNodeBase::OBSTACLE < maps::grid::TraversabilityNodeBase::FRONTIER + 1, "");
    static_assert(maps::grid::TraversabilityNodeBase::TRAVERSABLE < maps::grid::TraversabilityNodeBase::FRONTIER + 1, "");
    static_assert(maps::grid::TraversabilityNodeBase::UNKNOWN < maps::grid::TraversabilityNodeBase::FRONTIER + 1, "");
    static_assert(maps::grid::TraversabilityNodeBase::HOLE < maps::grid::TraversabilityNodeBase::FRONTIER + 1, "");
    static_assert(maps::grid::TraversabilityNodeBase::UNSET < maps::grid::TraversabilityNodeBase::FRONTIER + 1, "");
    static_assert(maps::grid::TraversabilityNodeBase::FRONTIER < maps::grid::TraversabilityNodeBase::FRONTIER + 1, "");
    
    
    minDistance.resize(maps::grid::TraversabilityNodeBase::FRONTIER + 1, std::numeric_limits< double >::max());
    
    
    minDistToObstacle = std::numeric_limits< double >::max();
}

void ugv_nav4d::PathStatistic::Stats::updateStatistic(const maps::grid::TraversabilityNodeBase* node)
{
    switch(node->getType())
    {
        case maps::grid::TraversabilityNodeBase::TRAVERSABLE:
            break;
        case maps::grid::TraversabilityNodeBase::FRONTIER:
            frontiers++;
            break;
        default:
            minDistToObstacle = std::min(minDistance[node->getType()], minDistToObstacle);
            obstacles++;
            break;
    }
}

void ugv_nav4d::PathStatistic::Stats::updateDistance(const maps::grid::TraversabilityNodeBase* node, double distance)
{
    minDistance[node->getType()] = std::min(minDistance[node->getType()], distance);
}

double ugv_nav4d::PathStatistic::Stats::getMinDistToFrontiers() const
{
    return minDistance[maps::grid::TraversabilityNodeBase::FRONTIER];
}

double ugv_nav4d::PathStatistic::Stats::getMinDistToObstacles() const
{
    return minDistToObstacle;
}

ugv_nav4d::PathStatistic::PathStatistic(const traversability_generator3d::TraversabilityConfig& config) : 
        config(config)
{
}

void ugv_nav4d::PathStatistic::calculateStatistics(const std::vector<const traversability_generator3d::TravGenNode* >& path, 
                                                   const std::vector< base::Pose2D >& poses, 
                                                   const traversability_generator3d::TravMap3d& trMap,
                                                   const std::string &debugObstacleName)
{
    assert(path.size() == poses.size());
//     CLEAR_DRAWING("CollisionBox");
   
    const Eigen::Vector2d travGridResolution(config.gridResolution, config.gridResolution);

    Eigen::Vector2d halfRobotDimension(config.robotSizeX / 2.0, config.robotSizeY/2.0);
    Eigen::Vector2d halfOuterBoxDimension(halfRobotDimension + Eigen::Vector2d::Constant(config.costFunctionDist));
    
    std::vector<Eigen::Vector2d> edgePositions = {
        Eigen::Vector2d(- config.gridResolution /2.0, - config.gridResolution / 2.0),
        Eigen::Vector2d(- config.gridResolution / 2.0, config.gridResolution / 2.0),
        Eigen::Vector2d(config.gridResolution / 2.0, config.gridResolution / 2.0),
        Eigen::Vector2d(config.gridResolution / 2.0, -config.gridResolution / 2.0)
    };
    
    Eigen::AlignedBox<double, 2> robotBoundingBox(- halfRobotDimension, halfRobotDimension);
    Eigen::AlignedBox<double, 2> costFunctionBoundingBox(- halfOuterBoxDimension, halfOuterBoxDimension);

    std::unordered_set<const maps::grid::TraversabilityNodeBase*> inRobot;
    std::unordered_set<const maps::grid::TraversabilityNodeBase*> inBoundary;

    for(size_t i = 0; i < path.size(); i++)
    {
        const traversability_generator3d::TravGenNode *node(path[i]);
        const base::Pose2D curPose(poses[i]);

        const Eigen::Rotation2D<double> yawInverse(Eigen::Rotation2D<double>(curPose.orientation).inverse());

        maps::grid::Vector3d nodePos3 = node->getPosition(trMap);
        const maps::grid::Vector2d nodePos(nodePos3.head<2>());

        bool hasObstacle = false;
        
        node->eachConnectedNode([&] (const maps::grid::TraversabilityNodeBase *neighbor, bool &explandNode, bool &stop){
            //we need to compute the four edges of a cell and check if any is inside of the robot
            maps::grid::Vector3d neighborPos;
            trMap.fromGrid(neighbor->getIndex(), neighborPos, neighbor->getHeight(), false);
            
            bool isInsideRobot = false;
            bool isInsideOuterBox = false;
            
            for(const Eigen::Vector2d &ep : edgePositions)
            {
                const Eigen::Vector2d edgePos(neighborPos.head<2>() + ep);
                
                //translate to center
                Eigen::Vector2d tp = edgePos - curPose.position;
                //rotate invers to orientation of robot
                tp = yawInverse * tp;
                
                if(costFunctionBoundingBox.contains(tp))
                {
                    isInsideOuterBox = true;
                    
                    //it is only inside the robot
                    if(robotBoundingBox.contains(tp))
                    {
                        isInsideRobot = true;
                        robotStats.updateDistance(neighbor, (nodePos3 - neighborPos).head<2>().norm());
                        //we continue iteration here, to compute the correct distances of all edges
                    }
                    else
                    {
                        boundaryStats.updateDistance(neighbor, robotBoundingBox.exteriorDistance(tp));
                    }
                }
            }
                
            if(!isInsideOuterBox)
            {
                return;
            }

            //further expand node
            explandNode = true;

            if(isInsideRobot)
            {
                inRobot.insert(neighbor);
                
                if(neighbor->getType() != maps::grid::TraversabilityNodeBase::TRAVERSABLE)
                {
#ifdef ENABLE_DEBUG_DRAWINGS                    
                    V3DD::COMPLEX_DRAWING([&]()
                    {
                        if(!debugObstacleName.empty())
                        {
                        V3DD::DRAW_ARROW(debugObstacleName,
                                         neighborPos,
                                         Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())), Eigen::Vector3d(.3, 0.3, 0.8), V3DD::Color::red);
                        }
                    });
#endif                    
                    hasObstacle = true;
                    stop = true;
                }
                
                return;
            }
            
            inBoundary.insert(neighbor);
            
        });
        /*
        if(hasObstacle)
        {
            COMPLEX_DRAWING (

            Eigen::Vector3d checkPos;
            checkPos.head<2>() = curPose.position;
            checkPos.z() = node->getHeight();
            
            DRAW_WIREFRAME_BOX("CollisionBox", checkPos, Eigen::Quaterniond(Eigen::AngleAxisd(curPose.orientation, Eigen::Vector3d::UnitZ())), Eigen::Vector3d(config.robotSizeX, config.robotSizeY, config.robotHeight), vizkit3dDebugDrawings::Color::red);
            );
        }*/

        if(hasObstacle)
            break;
    }

    for(const maps::grid::TraversabilityNodeBase* n : inBoundary)
    {
        if(inRobot.find(n) == inRobot.end())
        {
            boundaryStats.updateStatistic(n);
        }
    }   
    
    for(const maps::grid::TraversabilityNodeBase* n : inRobot)
    {
        robotStats.updateStatistic(n);
    }   

}

bool ugv_nav4d::PathStatistic::isPathFeasible(const std::vector<const traversability_generator3d::TravGenNode* >& path, 
                                                   const std::vector< base::Pose2D >& poses, 
                                                   const traversability_generator3d::TravMap3d& trMap)
{    
    assert(path.size() == poses.size());
 
    const Eigen::Vector2d travGridResolution(config.gridResolution, config.gridResolution);

    Eigen::Vector2d halfRobotDimension(config.robotSizeX / 2.0, config.robotSizeY/2.0);
    Eigen::Vector2d halfOuterBoxDimension(halfRobotDimension + Eigen::Vector2d::Constant(config.costFunctionDist));
    
    std::vector<Eigen::Vector2d> edgePositions = {
        Eigen::Vector2d(- config.gridResolution /2.0, - config.gridResolution / 2.0),
        Eigen::Vector2d(- config.gridResolution / 2.0, config.gridResolution / 2.0),
        Eigen::Vector2d(config.gridResolution / 2.0, config.gridResolution / 2.0),
        Eigen::Vector2d(config.gridResolution / 2.0, -config.gridResolution / 2.0)
    };
    
    Eigen::AlignedBox<double, 2> robotBoundingBox(- halfRobotDimension, halfRobotDimension);
    Eigen::AlignedBox<double, 2> costFunctionBoundingBox(- halfOuterBoxDimension, halfOuterBoxDimension);

    std::unordered_set<const maps::grid::TraversabilityNodeBase*> inRobot;
    std::unordered_set<const maps::grid::TraversabilityNodeBase*> inBoundary;

    for(size_t i = 0; i < path.size(); i++)
    {
        const traversability_generator3d::TravGenNode *node(path[i]);
        const base::Pose2D curPose(poses[i]);

        const Eigen::Rotation2D<double> yawInverse(Eigen::Rotation2D<double>(curPose.orientation).inverse());

        maps::grid::Vector3d nodePos3 = node->getPosition(trMap);
        const maps::grid::Vector2d nodePos(nodePos3.head<2>());

        bool hasObstacle = false;
 
        node->eachConnectedNode([&] (const maps::grid::TraversabilityNodeBase *neighbor, bool &explandNode, bool &stop){
            //we need to compute the four edges of a cell and check if any is inside of the robot
            maps::grid::Vector3d neighborPos;
            trMap.fromGrid(neighbor->getIndex(), neighborPos, neighbor->getHeight(), false);
            
            bool isInsideRobot = false;
            bool isInsideOuterBox = false;
            
            for(const Eigen::Vector2d &ep : edgePositions)
            {
                const Eigen::Vector2d edgePos(neighborPos.head<2>() + ep);
                
                //translate to center
                Eigen::Vector2d tp = edgePos - curPose.position;
                //rotate invers to orientation of robot
                tp = yawInverse * tp;
                
                if(costFunctionBoundingBox.contains(tp))
                {
                    isInsideOuterBox = true;
                    
                    //it is only inside the robot
                    if(robotBoundingBox.contains(tp))
                    {
                        isInsideRobot = true;
                        break;
                    }
                }
            }
                
            if(!isInsideOuterBox)
            {
                return;
            }

            //further expand node
            explandNode = true;

            if(isInsideRobot)
            {
               if(neighbor->getType() != maps::grid::TraversabilityNodeBase::TRAVERSABLE)
                {
                    hasObstacle = true;
                    stop = true;
                }
                
                return;
            }
         
        });
        if(hasObstacle)
            return false;
    }
    return true;
}
