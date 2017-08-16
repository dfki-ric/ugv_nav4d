#include "PathStatistics.hpp"
#include <unordered_set>
#include <deque>

ugv_nav4d::PathStatistic::Stats::Stats() :
    obstacles(0), 
    frontiers(0)
{
    //FIXME this is going to break if FRONTIER is no longer the last entry in the enum
    //      or someone starts attaching different values to the enums.
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


ugv_nav4d::PathStatistic::PathStatistic(const ugv_nav4d::TraversabilityConfig& config) : 
        config(config)
{
}

void ugv_nav4d::PathStatistic::calculateStatistics(const std::vector< ugv_nav4d::TravGenNode* >& path, 
                                                   const std::vector< base::Pose2D >& poses, 
                                                   const maps::grid::TraversabilityMap3d<TravGenNode *> &trMap)
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

    std::unordered_set<maps::grid::TraversabilityNodeBase*> inRobot;
    std::unordered_set<maps::grid::TraversabilityNodeBase*> inBoundary;


    
    for(size_t i = 0; i < path.size(); i++)
    {
        TravGenNode *node(path[i]);
        const base::Pose2D curPose(poses[i]);

        const Eigen::Rotation2D<double> yawInverse(Eigen::Rotation2D<double>(curPose.orientation).inverse());

        
        //vector is not the the most efficient when using std::find but for small vectors it should be ok.
        //linear serach on a cached vector is as fast as unordered_set lookup for vector sizes < 100
        // (yes, I benchmarked)
        std::deque<maps::grid::TraversabilityNodeBase*> nodes;
        std::unordered_set<maps::grid::TraversabilityNodeBase*> visited;
        nodes.push_back(node);
        maps::grid::Vector3d nodePos3;
        //index check was already performed before
        trMap.fromGrid(node->getIndex(), nodePos3, false);
        
        const maps::grid::Vector2d nodePos(nodePos3.head<2>());

        do
        {
            maps::grid::TraversabilityNodeBase* currentNode = nodes.front();
            nodes.pop_front();
            
            for(auto neighbor : currentNode->getConnections())
            {
                //check if we have already visited this node (happens because double connected graph)
                if(visited.find(neighbor) != visited.end())
                    continue;

                visited.insert(neighbor);
                    
                
                //we need to compute the four edges of a cell and check if any is inside of the robot
                maps::grid::Vector3d neighborPos;
                trMap.fromGrid(neighbor->getIndex(), neighborPos, false);
                
                bool isInsideRobot = false;
                bool isInsideBoundary = false;
                
                for(const Eigen::Vector2d &ep : edgePositions)
                {
                    const Eigen::Vector2d edgePos(neighborPos.head<2>() + ep);
                    
                    //translate to center
                    Eigen::Vector2d tp = edgePos - curPose.position;
                    //rotate invers to orientation of robot
                    tp = yawInverse * tp;
                    
                    if(costFunctionBoundingBox.contains(tp))
                    {
                        isInsideBoundary = true;
                        
                        //it is only inside the robot
                        if(robotBoundingBox.contains(tp))
                        {
                            isInsideRobot = true;
                            
                            //we continue iteration here, to compute the correct distances of all edges
                        }
                        else
                        {
                            boundaryStats.updateDistance(neighbor, robotBoundingBox.exteriorDistance(tp));
                        }
                    }
                }
                    
                if(!isInsideBoundary)
                {
                    continue;
                }

                //further expand node
                nodes.push_back(neighbor);

                if(isInsideRobot)
                {
                    inRobot.insert(neighbor);
                    continue;
                }
                
                inBoundary.insert(neighbor);
            }
        }while(!nodes.empty());
    }
    
    for(maps::grid::TraversabilityNodeBase* n : inRobot)
    {
        robotStats.updateStatistic(n);
    }   

    for(maps::grid::TraversabilityNodeBase* n : inBoundary)
    {
        boundaryStats.updateStatistic(n);
    }   
}
