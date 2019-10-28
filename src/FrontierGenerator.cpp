#include "FrontierGenerator.hpp"
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>
#include <vizkit3d_debug_drawings/DebugDrawingColors.hpp>
#include "TravMapBfsVisitor.hpp"
#include "CollisionCheck.hpp"
#include "Dijkstra.hpp"
#include <Eigen/Geometry>
#include "PathStatistics.hpp"
#include "OrientedBox.hpp"


#define SHOW(x) std::cout << #x": "<< (x) << std::endl

using base::samples::RigidBodyState;
using maps::grid::TraversabilityNodeBase;
namespace ugv_nav4d 
{

struct NodeWithOrientation
{
    const TravGenNode* node;
    double orientationZ;
};

struct MovedNode
{
    NodeWithOrientation movedFrom;
    const TravGenNode* newNode;
};

struct NodeWithOrientationAndCost
{
    const TravGenNode* node;
    double orientationZ;
    double cost;
};

    
    
FrontierGenerator::FrontierGenerator(const TraversabilityConfig& travConf,
                                     const CostFunctionParameters& costParams) :
    costParams(costParams), travConf(travConf), travGen(travConf),
    robotPos(0, 0, 0), goalPos(0, 0, 0)
{
}

void FrontierGenerator::setInitialPatch(const Eigen::Affine3d& body2Mls, double patchRadius)
{
    travGen.setInitialPatch(body2Mls, patchRadius);
}

void FrontierGenerator::updateGoalPos(const base::Vector3d& _goalPos)
{
    goalPos = _goalPos;
    V3DD::CLEAR_DRAWING("ugv_nav4d_goalPos");
    V3DD::DRAW_ARROW("ugv_nav4d_goalPos", _goalPos,
                     base::Quaterniond(Eigen::AngleAxisd(M_PI, base::Vector3d::UnitX())),
                     base::Vector3d(1,1,1), V3DD::Color::yellow);
}


void FrontierGenerator::updateRobotPos(const base::Vector3d& _robotPos)
{
    robotPos = _robotPos;
    V3DD::CLEAR_DRAWING("ugv_nav4d_RobotPos");
    V3DD::DRAW_ARROW("ugv_nav4d_RobotPos", _robotPos, base::Quaterniond(Eigen::AngleAxisd(M_PI, base::Vector3d::UnitX())),
                     base::Vector3d(1,1,1), V3DD::Color::blue);
    
    V3DD::CLEAR_DRAWING("ugv_nav4d_robotToGoal");
    V3DD::DRAW_LINE("ugv_nav4d_robotToGoal", robotPos, goalPos, V3DD::Color::magenta);
}


base::Vector3d FrontierGenerator::nodeCenterPos(const TravGenNode* node) const
{
    Eigen::Vector3d pos;
    travGen.getTraversabilityMap().fromGrid(node->getIndex(), pos, node->getHeight(), false);
    return pos;
}

    
std::vector<RigidBodyState> FrontierGenerator::getNextFrontiers()
{
    V3DD::CLEAR_DRAWING("ugv_nav4d_visitable");
    
    std::vector<RigidBodyState> result;
    
    TravGenNode* startNode = travGen.generateStartNode(robotPos);
    travGen.expandAll(startNode);
    
    std::cout << "find frontiers" << std::endl;
    std::vector<const TravGenNode*> frontier(getFrontierPatches());

    if(coverageMap)
    {
        std::cout << "Removing already covered poses" << std::endl;
        auto last_it = std::remove_if(frontier.begin(), frontier.end(), [&](const TravGenNode* node){
            const auto& cell = coverageMap->at(node->getIndex());
            for(const auto& patch : cell)  // this could be replaced by a find operation, but we will hardly have more than 1 or 2 patches per cell
            {
                if(patch.isCovered(node->getHeight(), 0.0f))
                {
                    return true;
                }
            }
            return false;
        }
        ); // remove_if
        std::cout << "Filtered out " << (frontier.end() - last_it) << " already visited patches\n";
        frontier.erase(last_it, frontier.end());
    }


    const std::vector<NodeWithOrientation> frontierWithOrientation(getFrontierOrientation(frontier));
    std::cout << "found frontiers: " << frontierWithOrientation.size() << std::endl;
    
    std::cout << "find candidates" << std::endl;
    const std::vector<NodeWithOrientation> candidatesWithOrientation(getCandidatesFromFrontierPatches(frontierWithOrientation));
    if(candidatesWithOrientation.size() == 0)
    {
        std::cout << "No candidates found" << std::endl;
        return result;
    }
    else
    {
        std::cout << "found candidates: " << candidatesWithOrientation.size() << std::endl;
    }
    
    std::cout << "Finding collision free neighbors" << std::endl;
    const std::vector<MovedNode> collisionFreeNeighbors(getCollisionFreeNeighbor(candidatesWithOrientation));
    std::cout << "Neighbors: " << collisionFreeNeighbors.size() << std::endl;
    
    std::cout << "Removing duplicates" << std::endl;
    const std::vector<MovedNode> nodesWithoutDuplicates(removeDuplicates(collisionFreeNeighbors));
    std::cout << "frontiers without duplicates: " << nodesWithoutDuplicates.size() << std::endl;
    
    std::cout << "calculating costs" << std::endl;
    const std::vector<NodeWithOrientationAndCost> nodesWithCost = calculateCost(startNode, goalPos, nodesWithoutDuplicates);
    std::cout << "calculated costs: " << nodesWithCost.size() << std::endl;
    
    std::cout << "sorting nodes" << std::endl;
    const std::vector<NodeWithOrientationAndCost> sortedNodes(sortNodes(nodesWithCost));
    std::cout << "sorted nodes: " << sortedNodes.size() << std::endl;
    
    std::cout << "Converting to poses" << std::endl;
    result = getPositions(sortedNodes);
    
    std::cout << "Done. position count: " << result.size() << std::endl;
    
    //test code:
    
    
    V3DD::COMPLEX_DRAWING([&]()
    {
        V3DD::CLEAR_DRAWING("ugv_nav4d_candidates");
        for(const auto& node : candidatesWithOrientation)
        {
            base::Vector3d pos(node.node->getIndex().x() * travGen.getTraversabilityMap().getResolution().x(), 
                               node.node->getIndex().y() * travGen.getTraversabilityMap().getResolution().y(),
                               node.node->getHeight());
            pos = travGen.getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
            V3DD::DRAW_CYLINDER("ugv_nav4d_candidates", pos + base::Vector3d(travGen.getTraversabilityMap().getResolution().x() / 2.0, travGen.getTraversabilityMap().getResolution().y() / 2.0, travGen.getTraversabilityMap().getResolution().x() / 2.0), base::Vector3d(0.05, 0.05, 2), V3DD::Color::blue);
        }
    });
    
    V3DD::COMPLEX_DRAWING([&]()
    {
        double maxCost = 0;
        double costSum = 0;
        V3DD::CLEAR_DRAWING("ugv_nav4d_explorable");  
        for(const auto& node : sortedNodes)
        {
            costSum += node.cost;
            if(node.cost > maxCost) 
                maxCost = node.cost;
        }
        for(const auto& node : sortedNodes)
        {
            const double value = (node.cost);// / maxCost) * 2;
            base::Vector3d pos(nodeCenterPos(node.node));
            pos.z() += value / 2.0;
            
            V3DD::DRAW_CYLINDER("ugv_nav4d_explorable", pos,  base::Vector3d(0.03, 0.03, value), V3DD::Color::green);
        }
    });
    
    
    V3DD::COMPLEX_DRAWING([&]()
    {
        V3DD::CLEAR_DRAWING("ugv_nav4d_frontierWithOrientation");
        for(const NodeWithOrientation& node : frontierWithOrientation)
        {
            Eigen::Vector3d pos(node.node->getIndex().x() * travGen.getTraversabilityMap().getResolution().x() + travGen.getTraversabilityMap().getResolution().x() / 2.0, node.node->getIndex().y() * travGen.getTraversabilityMap().getResolution().y() + travGen.getTraversabilityMap().getResolution().y() / 2.0, node.node->getHeight());
            pos = travGen.getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
            pos.z() += 0.02;
            const double radius = travGen.getTraversabilityMap().getResolution().x() / 2.0;
            V3DD::DRAW_RING("ugv_nav4d_frontierWithOrientation", pos, radius, 0.01, 0.01, V3DD::Color::blue);
            const Eigen::Rotation2Dd rot(node.orientationZ);
            Eigen::Vector2d rotVec(travGen.getTraversabilityMap().getResolution().x() / 2.0, 0);
            rotVec = rot * rotVec;
            Eigen::Vector3d to(pos);
            to.topRows(2) += rotVec;
            V3DD::DRAW_LINE("ugv_nav4d_frontierWithOrientation", pos, to, V3DD::Color::cyan);
        }
    });
     
//     V3DD::COMPLEX_DRAWING([&]()
//     {
//         V3DD::CLEAR_DRAWING("ugv_nav4d_nodesWithoutCollisions");
//         for(const NodeWithOrientation& node : nodesWithoutCollisions)
//         {
//             Eigen::Vector3d pos(node.node->getIndex().x() * travGen.getTraversabilityMap().getResolution().x() + travGen.getTraversabilityMap().getResolution().x() / 2.0, node.node->getIndex().y() * travGen.getTraversabilityMap().getResolution().y() + travGen.getTraversabilityMap().getResolution().y() / 2.0, node.node->getHeight());
//             pos = travGen.getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
//             pos.z() += 0.02;
//             const double radius = travGen.getTraversabilityMap().getResolution().x() / 2.0;
//             V3DD::DRAW_RING("ugv_nav4d_nodesWithoutCollisions", pos, radius, 0.01, 0.01, V3DD::Color::green);
//             const Eigen::Rotation2Dd rot(node.orientationZ);
//             Eigen::Vector2d rotVec(travGen.getTraversabilityMap().getResolution().x() / 2.0, 0);
//             rotVec = rot * rotVec;
//             Eigen::Vector3d to(pos);
//             to.topRows(2) += rotVec;
//             V3DD::DRAW_LINE("ugv_nav4d_nodesWithoutCollisions", pos, to, V3DD::Color::green);
//         }
//     });
    
    V3DD::COMPLEX_DRAWING([&]()
    {
        V3DD::CLEAR_DRAWING("ugv_nav4d_sortedNodes");
        for(size_t i = 0; i < sortedNodes.size(); ++i)
        {
            const NodeWithOrientationAndCost& node = sortedNodes[i];
            Eigen::Vector3d pos(node.node->getIndex().x() * travGen.getTraversabilityMap().getResolution().x() + travGen.getTraversabilityMap().getResolution().x() / 2.0, node.node->getIndex().y() * travGen.getTraversabilityMap().getResolution().y() + travGen.getTraversabilityMap().getResolution().y() / 2.0, node.node->getHeight());
            pos = travGen.getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
            pos.z() += 0.02;
            
            V3DD::DRAW_TEXT("ugv_nav4d_sortedNodes", pos, std::to_string(i), 0.3, V3DD::Color::magenta);
        }
    });
     
    
    return std::move(result);
}




 
std::vector<const TravGenNode*> FrontierGenerator::getFrontierPatches() const
{
    std::vector<const TravGenNode*> frontier;
    for(const maps::grid::LevelList<TravGenNode*>& list : travGen.getTraversabilityMap())
    {
        for(const TravGenNode* node : list)
        {
            if(node->getType() == TraversabilityNodeBase::FRONTIER)
            {
                //check if the frontier borders on traversable, otherwise it is not reachable anyway
                
                for(TraversabilityNodeBase* neighbor : node->getConnections())
                {
                    if(neighbor->getType() == TraversabilityNodeBase::TRAVERSABLE)
                    {
                        frontier.push_back(node);
                        break;
                    }
                }
                
                
            }
        }
    }  
    return std::move(frontier);    
}

std::vector<NodeWithOrientation> FrontierGenerator::getCandidatesFromFrontierPatches(const std::vector<NodeWithOrientation> &frontiers) const
{
    std::vector<NodeWithOrientation> candidates;
    
    for(const NodeWithOrientation& node : frontiers)
    {
        for(const TraversabilityNodeBase *connected : node.node->getConnections())
        {
            if(connected->getType() == TraversabilityNodeBase::TRAVERSABLE)
            {
                candidates.push_back(NodeWithOrientation{reinterpret_cast<const TravGenNode *>(connected), node.orientationZ});
            }
        }
    }
    return std::move(candidates); 
}


std::vector<NodeWithOrientation> FrontierGenerator::getFrontierOrientation(const std::vector<const TravGenNode*>& frontier) const
{
//     CLEAR_DRAWING("edge direction");
//     CLEAR_DRAWING("robotToPatch");

    std::vector<NodeWithOrientation> frontierWithOrientation;
    for(const TravGenNode* frontierPatch : frontier)
    {
        const Eigen::Vector3d patchPos = frontierPatch->getPosition(travGen.getTraversabilityMap());
        const Eigen::Vector3d robotToPatch = patchPos - robotPos;
        
//         DRAW_LINE("robotToPatch", robotPos, robotPos + robotToPatch, vizkit3dDebugDrawings::Color::cyan);
        
        base::Angle orientation = base::Angle::fromRad(atan2(robotToPatch.y(), robotToPatch.x()));
        
        //check if frontier is allowed, if not use the closest allowed frontier
        bool orientationAllowed = false;
        for(const base::AngleSegment& allowedOrientation : frontierPatch->getUserData().allowedOrientations)
        {
            if(allowedOrientation.isInside(orientation))
            {
                orientationAllowed = true;
                break;
            }
        }
        if(!orientationAllowed)
        {
            const base::AngleSegment& firstSegment = frontierPatch->getUserData().allowedOrientations[0];
            orientation = firstSegment.getStart();
            orientation += base::Angle::fromRad(firstSegment.getWidth() / 2.0);
        }
        frontierWithOrientation.push_back(NodeWithOrientation{frontierPatch, orientation.getRad()});

//          COMPLEX_DRAWING(
//             Eigen::AngleAxisd rot(orientation.getRad(), Eigen::Vector3d::UnitZ());            
//             Eigen::Vector3d end = rot * Eigen::Vector3d(travGen.getTraversabilityMap().getResolution().x() , 0, frontierPatch->getHeight());
//             DRAW_LINE("edge direction", patchPos, patchPos + end, vizkit3dDebugDrawings::Color::red);
//         );
    }
    return std::move(frontierWithOrientation);
}


std::vector<MovedNode> FrontierGenerator::getCollisionFreeNeighbor(const std::vector<NodeWithOrientation>& nodes) const
{
    std::vector<MovedNode> result;
    V3DD::CLEAR_DRAWING("ugv_nav4d_neighborObstacleCheck");
    
    for(const NodeWithOrientation& node : nodes)
    {
        
        maps::grid::Vector3d nodePos;
        travGen.getTraversabilityMap().fromGrid(node.node->getIndex(), nodePos, node.node->getHeight(), false);
        
        const TravGenNode* traversableNeighbor = nullptr;
        TravMapBfsVisitor::visit(node.node, 
            [this, &traversableNeighbor, &nodePos, &node]
            (const TravGenNode* currentNode, bool& visitChildren, bool& abort, std::size_t distToRoot)
            {
                maps::grid::Vector3d neighborPos;
                travGen.getTraversabilityMap().fromGrid(currentNode->getIndex(), neighborPos, currentNode->getHeight(), false);
                
                if(currentNode->getType() == maps::grid::TraversabilityNodeBase::TRAVERSABLE)
                {
                    const base::Pose2D pose(neighborPos.topRows(2), node.orientationZ);
                    PathStatistic stats(travConf);
                    std::vector<const TravGenNode*> path;
                    path.push_back(currentNode);
                    std::vector<base::Pose2D> poses;
                    poses.push_back(pose);
                    stats.calculateStatistics(path, poses, travGen.getTraversabilityMap());
                    
                    if(!stats.getRobotStats().getNumObstacles() && !stats.getRobotStats().getNumFrontiers())
                    {
                        //found a patch that the robot can stand on without collision.
                        traversableNeighbor = currentNode;
                        abort = true;
                    }
                    else
                    {
                        abort = false;
                    }
                }
                else
                {
                    abort = false;
                }
                
                if(!abort)
                {
                    V3DD::DRAW_CYLINDER("ugv_nav4d_neighborObstacleCheck", neighborPos, base::Vector3d(0.05, 0.05, 2), V3DD::Color::red);
                    
                    const double dist = (nodePos - neighborPos).norm();
                    if(dist < travConf.robotSizeX + travConf.gridResolution)
                        visitChildren = true;
                    else
                        visitChildren = false;
                }
                
            });
                
        if(traversableNeighbor != nullptr)
        {
            MovedNode movedNode;
            movedNode.movedFrom = node;
            movedNode.newNode = traversableNeighbor;
            result.push_back(movedNode);
        }
    }

    return std::move(result);
}


std::vector<MovedNode> FrontierGenerator::removeDuplicates(const std::vector<MovedNode>& nodes) const
{
    //FIXME probably performance could be improved a lot
    std::vector<MovedNode> result;
    std::unordered_set<const TravGenNode*> set;
    for(const MovedNode& node : nodes)
    {
        if(set.find(node.newNode) == set.end())
        {
            set.insert(node.newNode);
            result.push_back(node);
        }
    }
    return result;
}


std::vector<NodeWithOrientationAndCost> FrontierGenerator::sortNodes(const std::vector<NodeWithOrientationAndCost>& nodes) const
{
    std::vector<NodeWithOrientationAndCost> result(nodes);
    std::sort(result.begin(), result.end(), 
        [](const NodeWithOrientationAndCost& a, const NodeWithOrientationAndCost& b)
        {
            return a.cost < b.cost;
        });
    return std::move(result);
}

std::vector<RigidBodyState> FrontierGenerator::getPositions(const std::vector<NodeWithOrientationAndCost>& nodes) const
{
    std::vector<RigidBodyState> result;
    const maps::grid::TraversabilityMap3d<TravGenNode *> &map(travGen.getTraversabilityMap());
    for(const NodeWithOrientationAndCost& node : nodes)
    {
        RigidBodyState rbs;
        Eigen::Vector3d pos;
        map.fromGrid(node.node->getIndex(), pos, node.node->getHeight(), false);
        rbs.position = pos;
        rbs.orientation = Eigen::AngleAxisd(node.orientationZ, Eigen::Vector3d::UnitZ());
        result.push_back(rbs);
    }
    return std::move(result);
}

std::vector<NodeWithOrientationAndCost> FrontierGenerator::calculateCost(const TravGenNode* startNode,
                                                                         const base::Vector3d& goalPos,
                                                                         const std::vector<MovedNode>& nodes) const
{
    //calc travel distances on map
    std::vector<NodeWithOrientationAndCost> result;
    std::vector<MovedNode> validNodes;
    std::unordered_map<const maps::grid::TraversabilityNodeBase*, double> distancesOnMap;
    Dijkstra::computeCost(startNode, distancesOnMap, travConf);
    
    //find max distances for normalization
    double maxDistFromStart = 0;
    double maxDistToGoal = 0;
    for(const MovedNode& node : nodes)
    {
        if(distancesOnMap.find(node.newNode) == distancesOnMap.end())
        {
            //this means there is no traversable connection to the node.
            continue;
        }
        
        const double distToGoal = distToPoint(node.newNode, goalPos);
        const double distFromStart = distancesOnMap[node.newNode];

        maxDistToGoal = std::max(maxDistToGoal, distToGoal);
        maxDistFromStart = std::max(maxDistFromStart, distFromStart);
        validNodes.push_back(node);
    }

    //calc cost
    for(MovedNode& node : validNodes)
    {
        const double distFromStart = distancesOnMap[node.newNode];
        const double distToGoal = distToPoint(node.newNode, goalPos) / maxDistToGoal; //range 0..1
        const double explorableFactor = calcExplorablePatches(node.movedFrom.node); //range: 0.. 1  //explorableFactor does only make sense for the original node because it is the one at the frontier
        const double travelDist = distFromStart / maxDistFromStart; //range: 0..1
        
        assert(distToGoal >= 0 && distToGoal <= 1);
        assert(explorableFactor >= 0 && explorableFactor <= 1);
        assert(travelDist >= 0 && travelDist <= 1);
        
        const double cost = costParams.distToGoalFactor * distToGoal +
                            costParams.explorableFactor * explorableFactor +
                            costParams.distFromStartFactor * travelDist;
        
                            
        NodeWithOrientationAndCost costNode;
        costNode.cost = cost;
        costNode.node = node.newNode;
        costNode.orientationZ = node.movedFrom.orientationZ;
        result.push_back(costNode);
    }
    return result;
}


double FrontierGenerator::distToPoint(const TravGenNode* node, const base::Vector3d& p) const
{
    const base::Vector3d nodePos(nodeCenterPos(node));
    return (nodePos - p).norm();
}



double FrontierGenerator::calcExplorablePatches(const TravGenNode* node) const
{
    std::size_t visited = 0;
    const size_t visitRadius = 3; //FIXME should be parameter
    /* Since the grid is a square we can calculate the number of visitable nodes using odd square*/
    const std::size_t maxVisitable = std::pow(2 * visitRadius + 1, 2);
    TravMapBfsVisitor::visit(node, 
        [&visited] (const TravGenNode* currentNode, bool& visitChildren, bool& abort, std::size_t distToRoot)
        {
            if(currentNode->getType() != TraversabilityNodeBase::UNKNOWN && currentNode->getType() != TraversabilityNodeBase::UNSET
               && currentNode->getType() != TraversabilityNodeBase::FRONTIER)
                ++visited;
            
            abort = false;
            if(distToRoot >= visitRadius)
                visitChildren = false;
            else
                visitChildren = true;
        });
    
    assert(visited <= maxVisitable);
    
    const double explorablePatches = (visited / (double)maxVisitable);
    
    V3DD::COMPLEX_DRAWING([&]()
    {
        Eigen::Vector3d pos(node->getIndex().x() * travGen.getTraversabilityMap().getResolution().x() + travGen.getTraversabilityMap().getResolution().x() / 2.0, node->getIndex().y() * travGen.getTraversabilityMap().getResolution().y() + travGen.getTraversabilityMap().getResolution().y() / 2.0, node->getHeight());
        pos = travGen.getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
        pos.z() += 0.02;
        
        V3DD::DRAW_TEXT("ugv_nav4d_visitable", pos, std::to_string(maxVisitable - visited), 0.3, V3DD::Color::magenta);
    });
    
    
    return explorablePatches;
}

void FrontierGenerator::updateCostParameters(const CostFunctionParameters& params)
{
    costParams = params;
}

const maps::grid::TraversabilityMap3d< TravGenNode* >& FrontierGenerator::getTraversabilityMap() const
{
    return travGen.getTraversabilityMap();
}

const TraversabilityConfig& FrontierGenerator::getConfig() const
{
    return travConf;
}

bool FrontierGenerator::patchesInBox(const OrientedBox& box) const
{
    size_t numIntersections = 0;
    travGen.getTraversabilityMap().intersectCuboid(box.getBox(), numIntersections);
    return numIntersections > 0;
}



}
