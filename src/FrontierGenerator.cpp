#include "FrontierGenerator.hpp"
#include <vizkit3d_debug_drawings/DebugDrawing.h>
#include <vizkit3d_debug_drawings/DebugDrawingColors.h>
#include "TravMapBfsVisitor.hpp"
#include "CollisionCheck.hpp"
#include <Eigen/Geometry>


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
    CLEAR_DRAWING("goalPos");
    DRAW_ARROW("goalPos", _goalPos, base::Quaterniond(Eigen::AngleAxisd(M_PI, base::Vector3d::UnitX())),
               base::Vector3d(1,1,1), vizkit3dDebugDrawings::Color::yellow);
    
    CLEAR_DRAWING("robotToGoal");
    DRAW_LINE("robotToGoal", robotPos, goalPos, vizkit3dDebugDrawings::Color::magenta);
    
}


void FrontierGenerator::updateRobotPos(const base::Vector3d& _robotPos)
{
    robotPos = _robotPos;
    CLEAR_DRAWING("RobotPos");
    DRAW_ARROW("RobotPos", _robotPos, base::Quaterniond(Eigen::AngleAxisd(M_PI, base::Vector3d::UnitX())),
               base::Vector3d(1,1,1), vizkit3dDebugDrawings::Color::blue);
    
    CLEAR_DRAWING("robotToGoal");
    DRAW_LINE("robotToGoal", robotPos, goalPos, vizkit3dDebugDrawings::Color::magenta);
}


base::Vector3d FrontierGenerator::nodeCenterPos(const TravGenNode* node) const
{
    Eigen::Vector3d pos;
    travGen.getTraversabilityMap().fromGrid(node->getIndex(), pos, node->getHeight(), false);
    return pos;
}

    
std::vector<RigidBodyState> FrontierGenerator::getNextFrontiers()
{
    CLEAR_DRAWING("visitable");
    
    std::vector<RigidBodyState> result;
    
    TravGenNode* startNode = travGen.generateStartNode(robotPos);
    travGen.expandAll(startNode);
    
    std::cout << "find frontiers" << std::endl;
    const std::vector<const TravGenNode*> frontier(getFrontierPatches());
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
    
    std::cout << "Removing frontiers with collisions" << std::endl;
    const std::vector<NodeWithOrientation> nodesWithoutCollisions(getNodesWithoutCollision(candidatesWithOrientation));
    std::cout << "frontiers without collision: " << nodesWithoutCollisions.size() << std::endl;
    
    std::cout << "Removing duplicates" << std::endl;
    const std::vector<NodeWithOrientation> nodesWithoutDuplicates(removeDuplicates(nodesWithoutCollisions));
    std::cout << "frontiers without duplicates: " << nodesWithoutDuplicates.size() << std::endl;
    
    std::cout << "calculating costs" << std::endl;
    const std::vector<NodeWithOrientationAndCost> nodesWithCost = calculateCost(startNode, goalPos, nodesWithoutDuplicates);
    std::cout << "calculated costs: " << nodesWithCost.size() << std::endl;
    
    std::cout << "sorting ndoes" << std::endl;
    const std::vector<NodeWithOrientationAndCost> sortedNodes(sortNodes(nodesWithCost));
    std::cout << "sorted nodes: " << sortedNodes.size() << std::endl;
    
    std::cout << "Converting to poses" << std::endl;
    result = getPositions(sortedNodes);
    
    std::cout << "Done. position count: " << result.size() << std::endl;
    
    //test code:
    
    
    COMPLEX_DRAWING(
        CLEAR_DRAWING("candidates");
        for(const auto& node : candidatesWithOrientation)
        {
            base::Vector3d pos(node.node->getIndex().x() * travGen.getTraversabilityMap().getResolution().x(), 
                               node.node->getIndex().y() * travGen.getTraversabilityMap().getResolution().y(),
                               node.node->getHeight());
            pos = travGen.getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
            DRAW_CYLINDER("candidates", pos + base::Vector3d(travGen.getTraversabilityMap().getResolution().x() / 2.0, travGen.getTraversabilityMap().getResolution().y() / 2.0, travGen.getTraversabilityMap().getResolution().x() / 2.0), base::Vector3d(0.05, 0.05, 2), vizkit3dDebugDrawings::Color::blue);
        }
    );
    
      COMPLEX_DRAWING(
        double maxCost = 0;
        double costSum = 0;
        CLEAR_DRAWING("explorable");  
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
            
            DRAW_CYLINDER("explorable", pos,  base::Vector3d(0.03, 0.03, value), vizkit3dDebugDrawings::Color::green);
        }
      );
    
    
//     COMPLEX_DRAWING(
//         CLEAR_DRAWING("frontierWithOrientation");
//         for(const NodeWithOrientation& node : frontierWithOrientation)
//         {
//             Eigen::Vector3d pos(node.node->getIndex().x() * travGen.getTraversabilityMap().getResolution().x() + travGen.getTraversabilityMap().getResolution().x() / 2.0, node.node->getIndex().y() * travGen.getTraversabilityMap().getResolution().y() + travGen.getTraversabilityMap().getResolution().y() / 2.0, node.node->getHeight());
//             pos = travGen.getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
//             pos.z() += 0.02;
//             const double radius = travGen.getTraversabilityMap().getResolution().x() / 2.0;
//             DRAW_RING("frontierWithOrientation", pos, radius, 0.01, 0.01, vizkit3dDebugDrawings::Color::blue);
//             const Eigen::Rotation2Dd rot(node.orientationZ);
//             Eigen::Vector2d rotVec(travGen.getTraversabilityMap().getResolution().x() / 2.0, 0);
//             rotVec = rot * rotVec;
//             Eigen::Vector3d to(pos);
//             to.topRows(2) += rotVec;
//             DRAW_LINE("frontierWithOrientation", pos, to, vizkit3dDebugDrawings::Color::cyan);
//         }
//      );
     
//     COMPLEX_DRAWING(
//         CLEAR_DRAWING("nodesWithoutCollisions");
//         for(const NodeWithOrientation& node : nodesWithoutCollisions)
//         {
//             Eigen::Vector3d pos(node.node->getIndex().x() * travGen.getTraversabilityMap().getResolution().x() + travGen.getTraversabilityMap().getResolution().x() / 2.0, node.node->getIndex().y() * travGen.getTraversabilityMap().getResolution().y() + travGen.getTraversabilityMap().getResolution().y() / 2.0, node.node->getHeight());
//             pos = travGen.getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
//             pos.z() += 0.02;
//             const double radius = travGen.getTraversabilityMap().getResolution().x() / 2.0;
//             DRAW_RING("nodesWithoutCollisions", pos, radius, 0.01, 0.01, vizkit3dDebugDrawings::Color::green);
//             const Eigen::Rotation2Dd rot(node.orientationZ);
//             Eigen::Vector2d rotVec(travGen.getTraversabilityMap().getResolution().x() / 2.0, 0);
//             rotVec = rot * rotVec;
//             Eigen::Vector3d to(pos);
//             to.topRows(2) += rotVec;
//             DRAW_LINE("nodesWithoutCollisions", pos, to, vizkit3dDebugDrawings::Color::green);
//         }
//      );
    
//     COMPLEX_DRAWING(
//         CLEAR_DRAWING("sortedNodes");
//         for(size_t i = 0; i < sortedNodes.size(); ++i)
//         {
//             const NodeWithOrientationAndCost& node = sortedNodes[i];
//             Eigen::Vector3d pos(node.node->getIndex().x() * travGen.getTraversabilityMap().getResolution().x() + travGen.getTraversabilityMap().getResolution().x() / 2.0, node.node->getIndex().y() * travGen.getTraversabilityMap().getResolution().y() + travGen.getTraversabilityMap().getResolution().y() / 2.0, node.node->getHeight());
//             pos = travGen.getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
//             pos.z() += 0.02;
//             
//             DRAW_TEXT("sortedNodes", pos, std::to_string(i), 0.3, vizkit3dDebugDrawings::Color::magenta);
//         }
//     );
     
    

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
                frontier.push_back(node);
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
    //sobel filter is used to get an estimate of the edge direction
    const int yOp[3][3] = {{1,0,-1},
                           {2,0,-2},
                           {1,0,-1}};
    const int xOp[3][3] = {{1,2,1},
                           {0,0,0},
                           {-1,-2,-1}};
    
    CLEAR_DRAWING("edge direction");

    std::vector<NodeWithOrientation> frontierWithOrientation;
    for(const TravGenNode* frontierPatch : frontier)
    {
        int xSum = 0;
        int ySum = 0;
        const maps::grid::Index i = frontierPatch->getIndex();
        
        for(int x = -1; x < 2; ++x)
        {
            for(int y = -1; y < 2; ++y)
            {
                const maps::grid::Index neighborIndex(x + i.x(), y + i.y());
                if(travGen.getTraversabilityMap().inGrid(neighborIndex))
                {
                    const TravGenNode* neighbor = frontierPatch->getConnectedNode(neighborIndex);
                    if(neighbor != nullptr && neighbor->getType() == TraversabilityNodeBase::FRONTIER)
                    {
                        xSum += xOp[x + 1][y + 1];
                        ySum += yOp[x + 1][y + 1];
                    }
                }
            }
        } 
        
        base::Angle orientation = base::Angle::fromRad(atan2(ySum, xSum));
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

        COMPLEX_DRAWING(
        
            Eigen::Vector3d start(i.x() * travConf.gridResolution + travConf.gridResolution / 2.0, i.y() * travConf.gridResolution + travConf.gridResolution / 2.0, frontierPatch->getHeight());
            start = travGen.getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * start;
            Eigen::Vector3d end((i.x() + xSum) * travConf.gridResolution + travConf.gridResolution / 2.0, (i.y() + ySum) * travConf.gridResolution + travConf.gridResolution / 2.0, frontierPatch->getHeight());
            end = travGen.getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * end;
            DRAW_LINE("edge direction", start, end, vizkit3dDebugDrawings::Color::red);
        );
    }
    return std::move(frontierWithOrientation);
}


std::vector<NodeWithOrientation> FrontierGenerator::getNodesWithoutCollision(const std::vector<NodeWithOrientation>& nodes) const
{
    std::vector<NodeWithOrientation> result;
    const base::Vector3d robotHalfSize(travConf.robotSizeX / 2, travConf.robotSizeY / 2, travConf.robotHeight / 2);
    
    CLEAR_DRAWING("removed_due_to_collision");
    
    for(const NodeWithOrientation& node : nodes)
    {
        if(CollisionCheck::checkCollision(node.node, node.orientationZ, mlsMap, robotHalfSize, travGen))
        {
            result.push_back(node);
        }
        else
        {
            COMPLEX_DRAWING(
            base::Vector3d pos(node.node->getIndex().x() * travGen.getTraversabilityMap().getResolution().x(), 
                               node.node->getIndex().y() * travGen.getTraversabilityMap().getResolution().y(),
                               node.node->getHeight());
            pos = travGen.getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
            DRAW_CYLINDER("removed_due_to_collision", pos + base::Vector3d(travGen.getTraversabilityMap().getResolution().x() / 2.0, travGen.getTraversabilityMap().getResolution().y() / 2.0, travGen.getTraversabilityMap().getResolution().x() / 2.0), base::Vector3d(0.05, 0.05, 2), vizkit3dDebugDrawings::Color::magenta);
            );

        }
    }   
    
    

    
    return std::move(result);
}

std::vector<NodeWithOrientation> FrontierGenerator::getCollisionFreeNeighbor(const std::vector<NodeWithOrientation>& nodes) const
{
    std::vector<NodeWithOrientation> result;
    
    //FIXME the collisions are drawn inside CollisionCheck::checkCollision().
    //      If they are not cleared we will run out of memory after some time.

    const base::Vector3d robotHalfSize(travConf.robotSizeX / 2, travConf.robotSizeY / 2, travConf.robotHeight / 2);
//     CLEAR_DRAWING("neighBorobstacleCheck");
    for(const NodeWithOrientation& node : nodes)
    {
        const TravGenNode* traversableNeighbor = nullptr;
        base::Vector3d traversableNeighborPos(0, 0, 0);
        const TravGenNode* nextFrontierNode = node.node;
        base::Vector3d nextFrontierPos(nextFrontierNode->getIndex().x() * travGen.getTraversabilityMap().getResolution().x(), 
                                       nextFrontierNode->getIndex().y() * travGen.getTraversabilityMap().getResolution().y(),
                                       nextFrontierNode->getHeight());
        nextFrontierPos = travGen.getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * nextFrontierPos;
        const double orientation = node.orientationZ;

        CLEAR_DRAWING("collisions");
        TravMapBfsVisitor::visit(node.node, 
            [&traversableNeighbor, &nextFrontierNode, &nextFrontierPos, this, orientation, &traversableNeighborPos, &robotHalfSize]
            (const TravGenNode* currentNode, bool& visitChildren, bool& abort, std::size_t distToRoot)
            {
                if((currentNode->getType() == maps::grid::TraversabilityNodeBase::TRAVERSABLE ||
                   currentNode->getType() == TraversabilityNodeBase::FRONTIER) &&
                   CollisionCheck::checkCollision(currentNode, orientation, mlsMap, robotHalfSize, travGen))
                {
                    traversableNeighbor = currentNode;
                    
                    base::Vector3d pos(currentNode->getIndex().x() * travGen.getTraversabilityMap().getResolution().x(), 
                                       currentNode->getIndex().y() * travGen.getTraversabilityMap().getResolution().y(),
                                       currentNode->getHeight());
                    pos = travGen.getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
                    traversableNeighborPos = pos;
//                     DRAW_CYLINDER("neighBorobstacleCheck", pos + base::Vector3d(travGen.getTraversabilityMap().getResolution().x() / 2.0, travGen.getTraversabilityMap().getResolution().y() / 2.0, travGen.getTraversabilityMap().getResolution().x() / 2.0), base::Vector3d(0.05, 0.05, 2), vizkit3dDebugDrawings::Color::green);
//                     found a nearby node that we can stand on, abort
                    abort = true;
                    
                }
                else
                {
                    abort = false;
                    base::Vector3d pos(currentNode->getIndex().x() * travGen.getTraversabilityMap().getResolution().x(), 
                                       currentNode->getIndex().y() * travGen.getTraversabilityMap().getResolution().y(),
                                       currentNode->getHeight());
                    pos = travGen.getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
                    
                     DRAW_CYLINDER("neighBorobstacleCheck", pos + base::Vector3d(travGen.getTraversabilityMap().getResolution().x() / 2.0, travGen.getTraversabilityMap().getResolution().y() / 2.0, travGen.getTraversabilityMap().getResolution().x() / 2.0), base::Vector3d(0.05, 0.05, 2), vizkit3dDebugDrawings::Color::red);
                    
                    const double dist = (nextFrontierPos - pos).norm();
                    if(dist < maxNeighborDistance)
                        visitChildren = true;
                    else
                        visitChildren = false;
                }
            });
        
        if(traversableNeighbor != nullptr)
            result.push_back({traversableNeighbor, node.orientationZ});
    }

    return std::move(result);
}


std::vector<NodeWithOrientation> FrontierGenerator::removeDuplicates(const std::vector<NodeWithOrientation>& nodes) const
{
    //FIXME probably performance could be improved a lot
    std::vector<NodeWithOrientation> result;
    std::unordered_set<const TravGenNode*> set;
    for(const NodeWithOrientation& node : nodes)
    {
        if(set.find(node.node) == set.end())
        {
            set.insert(node.node);
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
                                                                         const std::vector<NodeWithOrientation>& nodes) const
{
    //calc travel distances on map
    std::vector<NodeWithOrientationAndCost> result;
    std::vector<double> distancesOnMap;
    const double maxDist = 99999999;//FIXME constant in code? should be numeric_limits::max
    travGen.dijkstraComputeCost(startNode, distancesOnMap, maxDist);
    
    
    //find max distances for normalization
    double maxDistFromStart = 0;
    double maxDistToGoal = 0;
    for(const NodeWithOrientation& node : nodes)
    {
        const double distToGoal = distToPoint(node.node, goalPos);
        const double distFromStart = distancesOnMap[node.node->getUserData().id];
        if(distFromStart >= maxDist)
        {
            std::cout << "DIST FROM START > MAX DIST" << std::endl;
            throw std::runtime_error("DIST FROM START > MAX DIST");
        }
        maxDistToGoal = std::max(maxDistToGoal, distToGoal);
        maxDistFromStart = std::max(maxDistFromStart, distFromStart);
    }

    //calc cost
    for(const NodeWithOrientation& node : nodes)
    {
        const double distFromStart = distancesOnMap[node.node->getUserData().id];
        if(distFromStart >= maxDist)
            continue;

        const double distToGoal = distToPoint(node.node, goalPos) / maxDistToGoal; //range 0..1
        const double explorableFactor = calcExplorablePatches(node.node); //range: 0.. 1
        const double travelDist = distFromStart / maxDistFromStart; //range: 0..1
        
        assert(distToGoal >= 0 && distToGoal <= 1);
        assert(explorableFactor >= 0 && explorableFactor <= 1);
        assert(travelDist >= 0 && travelDist <= 1);
        
        const double cost = costParams.distToGoalFactor * distToGoal +
                            costParams.explorableFactor * explorableFactor +
                            costParams.distFromStartFactor * travelDist;
        
        result.push_back({node.node, node.orientationZ, cost});
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
            ++visited;
            abort = false;
            if(distToRoot >= visitRadius)
                visitChildren = false;
            else
                visitChildren = true;
        });
    
    assert(visited <= maxVisitable);
    
    const double explorablePatches = (visited / (double)maxVisitable);
    
    COMPLEX_DRAWING(
        Eigen::Vector3d pos(node->getIndex().x() * travGen.getTraversabilityMap().getResolution().x() + travGen.getTraversabilityMap().getResolution().x() / 2.0, node->getIndex().y() * travGen.getTraversabilityMap().getResolution().y() + travGen.getTraversabilityMap().getResolution().y() / 2.0, node->getHeight());
        pos = travGen.getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
        pos.z() += 0.02;
        
        DRAW_TEXT("visitable", pos, std::to_string(maxVisitable - visited), 0.3, vizkit3dDebugDrawings::Color::magenta);
    );
    
    
    return explorablePatches;
}

void FrontierGenerator::updateCostParameters(const CostFunctionParameters& params)
{
    costParams = params;
}

maps::grid::TraversabilityMap3d< TraversabilityNodeBase* > FrontierGenerator::getTraversabilityBaseMap() const
{
    return travGen.getTraversabilityBaseMap();
}

const TraversabilityConfig& FrontierGenerator::getConfig() const
{
    return travConf;
}



}