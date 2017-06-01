#include "FrontierGenerator.hpp"
#include <vizkit3d_debug_drawings/DebugDrawing.h>
#include "TravMapBfsVisitor.hpp"

using base::samples::RigidBodyState;
using maps::grid::TraversabilityNodeBase;
namespace ugv_nav4d 
{

struct NodeWithOrientation
{
    const TravGenNode* node;
    double orientationZ;
};

    
    
FrontierGenerator::FrontierGenerator(const maps::grid::TraversabilityMap3d< TravGenNode* >& travMap,
                                     const EnvironmentXYZTheta& env) :
    travMap(travMap), env(env)
{

}

    
std::vector<RigidBodyState> FrontierGenerator::getNextFrontiers(const base::Vector3d& closeTo) const
{
    std::vector<RigidBodyState> result;
    const std::vector<const TravGenNode*> frontier(getFrontierPatches());

    if(frontier.size() == 0)
        return result;

    const std::vector<NodeWithOrientation> frontierWithOrientation(getFrontierOrientation(frontier));
    const std::vector<NodeWithOrientation> nodesWithoutCollisions(getCollisionFreeNeighbor(frontierWithOrientation));
    const std::vector<NodeWithOrientation> sortedNodes(sortNodes(nodesWithoutCollisions, closeTo));
    
    COMPLEX_DRAWING(
        for(const NodeWithOrientation& node : frontierWithOrientation)
        {
            Eigen::Vector3d pos(node.node->getIndex().x() * travMap.getResolution().x() + travMap.getResolution().x() / 2.0, node.node->getIndex().y() * travMap.getResolution().y() + travMap.getResolution().y() / 2.0, node.node->getHeight());
            pos = travMap.getLocalFrame().inverse(Eigen::Isometry) * pos;
            pos.z() += 0.02;
            const double radius = travMap.getResolution().x() / 2.0;
            DRAW_RING("frontierWithOrientation", pos, radius, 0.01, 0.01, vizkit3dDebugDrawings::Color::blue);
            const Eigen::Rotation2Dd rot(node.orientationZ);
            Eigen::Vector2d rotVec(travMap.getResolution().x() / 2.0, 0);
            rotVec = rot * rotVec;
            Eigen::Vector3d to(pos);
            to.topRows(2) += rotVec;
            DRAW_LINE("frontierWithOrientation", pos, to, vizkit3dDebugDrawings::Color::cyan);
        }
     );
     
    COMPLEX_DRAWING(
        for(const NodeWithOrientation& node : nodesWithoutCollisions)
        {
            Eigen::Vector3d pos(node.node->getIndex().x() * travMap.getResolution().x() + travMap.getResolution().x() / 2.0, node.node->getIndex().y() * travMap.getResolution().y() + travMap.getResolution().y() / 2.0, node.node->getHeight());
            pos = travMap.getLocalFrame().inverse(Eigen::Isometry) * pos;
            pos.z() += 0.02;
            const double radius = travMap.getResolution().x() / 2.0;
            DRAW_RING("nodesWithoutCollisions", pos, radius, 0.01, 0.01, vizkit3dDebugDrawings::Color::green);
            const Eigen::Rotation2Dd rot(node.orientationZ);
            Eigen::Vector2d rotVec(travMap.getResolution().x() / 2.0, 0);
            rotVec = rot * rotVec;
            Eigen::Vector3d to(pos);
            to.topRows(2) += rotVec;
            DRAW_LINE("nodesWithoutCollisions", pos, to, vizkit3dDebugDrawings::Color::green);
        }
     );
    
    COMPLEX_DRAWING(
        for(int i = 0; i < sortedNodes.size(); ++i)
        {
            const NodeWithOrientation& node = sortedNodes[i];
            Eigen::Vector3d pos(node.node->getIndex().x() * travMap.getResolution().x() + travMap.getResolution().x() / 2.0, node.node->getIndex().y() * travMap.getResolution().y() + travMap.getResolution().y() / 2.0, node.node->getHeight());
            pos = travMap.getLocalFrame().inverse(Eigen::Isometry) * pos;
            pos.z() += 0.02;
            
            DRAW_TEXT("sortedNodes", pos, std::to_string(i), 0.1, vizkit3dDebugDrawings::Color::magenta);
        }
    );
     
    
    
    
    
    /** 
     * (1) alle frontiers finden
     * (2) zu jeder frontier eine zielorientierung rausfinden
     * (3) zu jeder frontier einen befahrbaren patch in der nähe mit entsprechender zielorientierung rausfinden
     * (3.1) TODO (3) ist fischig. was passiert am hang wo die zielorientierung vielleicht einfach nicht zulässig ist?
     * (4) Die patches sortieren nach irgendeiner bewertungsfunktion
     * (5) Die Patches zusammenfassen? Patches die nah beinander sind zu einem machen?
     * */
    
    return std::move(result);
}

 
std::vector<const TravGenNode*> FrontierGenerator::getFrontierPatches() const
{
    std::vector<const TravGenNode*> frontier;
    for(const maps::grid::LevelList<TravGenNode*>& list : travMap)
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

std::vector<NodeWithOrientation> FrontierGenerator::getFrontierOrientation(const std::vector<const TravGenNode*>& frontier) const
{
    //FIXME determine orientation somehow
    std::vector<NodeWithOrientation> frontierWithOrientation;
    for(const TravGenNode* frontierPatch : frontier)
    {
        frontierWithOrientation.push_back(NodeWithOrientation{frontierPatch, 1.5708});
    }
    return std::move(frontierWithOrientation);
}

std::vector<NodeWithOrientation> FrontierGenerator::getCollisionFreeNeighbor(const std::vector<NodeWithOrientation>& nodes) const
{
    std::vector<NodeWithOrientation> result;
    
    for(const NodeWithOrientation& node : nodes)
    {
        const TravGenNode* traversableNeighbor = nullptr;
        base::Vector3d traversableNeighborPos(0, 0, 0);
        const TravGenNode* nextFrontierNode = node.node;
        base::Vector3d nextFrontierPos(nextFrontierNode->getIndex().x() * travMap.getResolution().x(), 
                                       nextFrontierNode->getIndex().y() * travMap.getResolution().y(),
                                       nextFrontierNode->getHeight());
        nextFrontierPos = travMap.getLocalFrame().inverse(Eigen::Isometry) * nextFrontierPos;
        const double orientation = node.orientationZ;

        TravMapBfsVisitor::visit(node.node, 
            [&traversableNeighbor, &nextFrontierNode, &nextFrontierPos, this, orientation, &traversableNeighborPos]
            (const TravGenNode* currentNode, bool& visitChildren, bool& abort)
            {
                if((currentNode->getType() == maps::grid::TraversabilityNodeBase::TRAVERSABLE ||
                   currentNode->getType() == TraversabilityNodeBase::FRONTIER) &&
                   env.checkCollision(currentNode, orientation))
                {
                    traversableNeighbor = currentNode;
                    
                    base::Vector3d pos(currentNode->getIndex().x() * travMap.getResolution().x(), 
                                       currentNode->getIndex().y() * travMap.getResolution().y(),
                                       currentNode->getHeight());
                    pos = travMap.getLocalFrame().inverse(Eigen::Isometry) * pos;
                    traversableNeighborPos = pos;
                    DRAW_CYLINDER("neighBorobstacleCheck", pos + base::Vector3d(travMap.getResolution().x() / 2.0, travMap.getResolution().y() / 2.0, travMap.getResolution().x() / 2.0), base::Vector3d(0.05, 0.05, 2), vizkit3dDebugDrawings::Color::green);
                    //found a nearby node that we can stand on, abort
                    abort = true;
                    
                }
                else
                {
                    abort = false;
                    base::Vector3d pos(currentNode->getIndex().x() * travMap.getResolution().x(), 
                                       currentNode->getIndex().y() * travMap.getResolution().y(),
                                       currentNode->getHeight());
                    pos = travMap.getLocalFrame().inverse(Eigen::Isometry) * pos;
                    
                    DRAW_CYLINDER("neighBorobstacleCheck", pos + base::Vector3d(travMap.getResolution().x() / 2.0, travMap.getResolution().y() / 2.0, travMap.getResolution().x() / 2.0), base::Vector3d(0.05, 0.05, 2), vizkit3dDebugDrawings::Color::red);
                    
                    const double dist = (nextFrontierPos - pos).norm();
                    if(dist < 1) //FIXME 1 = 1 meter? should be parameter?!
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

std::vector<NodeWithOrientation> FrontierGenerator::sortNodes(const std::vector<NodeWithOrientation>& nodes,
                                                                const base::Vector3d& closeTo) const
{
    std::vector<NodeWithOrientation> result(nodes);
    const double resX = travMap.getResolution().x();
    const double resY = travMap.getResolution().y();
    const auto transform = travMap.getLocalFrame().inverse(Eigen::Isometry);
    std::sort(result.begin(), result.end(), 
        [resX, resY, &transform, &closeTo](const NodeWithOrientation& a, const NodeWithOrientation& b)
        {
            base::Vector3d aPos(a.node->getIndex().x() * resX, 
                                a.node->getIndex().y() * resY,
                                a.node->getHeight());
            aPos = transform * aPos;
            base::Vector3d bPos(b.node->getIndex().x() * resX, 
                                b.node->getIndex().y() * resY,
                                b.node->getHeight());
            bPos = transform * bPos;
            
            const double aDist = (closeTo - aPos).squaredNorm();
            const double bDist = (closeTo - bPos).squaredNorm();
            return aDist < bDist;
        });
    return std::move(result);
}




}