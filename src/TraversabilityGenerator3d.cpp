#include "TraversabilityGenerator3d.hpp"
#include <numeric/PlaneFitting.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <vizkit3d_debug_drawings/DebugDrawing.h>
#include <vizkit3d_debug_drawings/DebugDrawingColors.h>

#include <deque>
using namespace maps::grid;

namespace ugv_nav4d
{

TraversabilityGenerator3d::TraversabilityGenerator3d(const TraversabilityConfig& config) : addInitialPatch(false), config(config)
{
    trMap.setResolution(Eigen::Vector2d(config.gridResolution, config.gridResolution));
    UGV_DEBUG(
        debugData.setTravConfig(config);
        debugData.setTravGen(this);
    )
}

TraversabilityGenerator3d::~TraversabilityGenerator3d()
{
    clearTrMap();
}

void TraversabilityGenerator3d::setInitialPatch(const Eigen::Affine3d& body2Mls, double distToGround, double patchRadius)
{
    Eigen::Affine3d body2Ground(Eigen::Affine3d::Identity());
    body2Ground.translation() = Eigen::Vector3d(0, 0, distToGround);
    initialPatch2Mls = body2Mls * body2Ground.inverse();
    addInitialPatch = true;
    
    this->patchRadius = patchRadius;
    
    if(mlsGrid)
        addInitialPatchToMLS();    
}

const maps::grid::TraversabilityMap3d<TravGenNode *> & TraversabilityGenerator3d::getTraversabilityMap() const
{
    return trMap;
}

int TraversabilityGenerator3d::getNumNodes() const
{
    return currentNodeId;
}


bool TraversabilityGenerator3d::computePlaneRansac(TravGenNode& node, const View &area)
{
    typedef pcl::PointXYZ PointT;
    
    pcl::PointCloud<PointT>::Ptr points(new pcl::PointCloud<PointT>());
    
    Eigen::Vector2d sizeHalf(area.getSize() / 2.0);
    
    double fX = area.getSize().x() / area.getNumCells().x();
    double fY = area.getSize().y() / area.getNumCells().y();
    
    int patchCnt = 0;
    
    for(size_t y = 0; y < area.getNumCells().y(); y++)
    {
        for(size_t x = 0; x < area.getNumCells().x(); x++)
        {
            Eigen::Vector3d pos;
            pos.x() = x * fX - sizeHalf.x();
            pos.y() = y * fY - sizeHalf.y();
            
            for(const SurfacePatchBase *p : area.at(x, y))
            {
                PointT pclP;
                pclP.z = p->getTop();
                pclP.x = pos.x();
                pclP.y = pos.y();
                points->push_back(pclP);
                
                patchCnt++;
            }
        }
    }

    //if less than 5 planes -> hole
    //TODO where to implement ? here or in check obstacles ?
    if(patchCnt < 5)
    {
        //ransac will not produce a result below 5 points
        return false;
    }

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (50);
    seg.setDistanceThreshold (0.1);

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (points);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () <= 5)
    {
//         std::cerr << "Could not estimate Ground Plane" << std::endl;
        return false;
    }


    Eigen::Vector3d normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    normal.normalize();
    double distToOrigin = coefficients->values[3];
    
    node.getUserData().plane = Eigen::Hyperplane<double, 3>(normal, distToOrigin);
    
    //adjust height of patch
    Eigen::ParametrizedLine<double, 3> line(Vector3d::Zero(), Eigen::Vector3d::UnitZ());
    Vector3d newPos =  line.intersectionPoint(node.getUserData().plane);
    
    if(newPos.x() > 0.0001 || newPos.y() > 0.0001)
        throw std::runtime_error("TraversabilityGenerator3d: Error, adjustement height calculation is weird");

    //remove and reeinter node
    auto &list(trMap.at(node.getIndex()));
    
    if(newPos.allFinite())
    {
        list.erase(&node);
        node.setHeight(newPos.z());
        list.insert(&node);
    }    
    
    const Eigen::Vector3d slopeDir = computeSlopeDirection(node.getUserData().plane);
    node.getUserData().slope = computeSlope(node.getUserData().plane);
    node.getUserData().slopeDirection = slopeDir;
    node.getUserData().slopeDirectionAtan2 = std::atan2(slopeDir.y(), slopeDir.x());
    
//     COMPLEX_DRAWING(
//         Eigen::Vector3d pos(node.getIndex().x() * config.gridResolution, node.getIndex().y() * config.gridResolution, node.getHeight());
//         pos = getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
//         pos.z() += 0.06;
//         DRAW_TEXT("slope", pos, std::to_string(node.getUserData().slope), 0.01, vizkit3dDebugDrawings::Color::red);
//     );
    
    UGV_DEBUG(
        debugData.planeComputed(node);
    ) 

    
    return true;
}

bool TraversabilityGenerator3d::computeAllowedOrientations(TravGenNode* node)
{

    if(node->getUserData().slope >= config.maxSlope)
        return false;
    
        
    if(node->getUserData().slope < config.inclineLimittingMinSlope) 
    {
        //all orientations allowed below the slope limit
        node->getUserData().allowedOrientations.emplace_back(base::Angle::fromRad(0), 2 * M_PI);
    }
    else
    {
        //only allow orientations 
        const double limitRad = interpolate(node->getUserData().slope, config.inclineLimittingMinSlope,
                                         M_PI_2, config.maxSlope, config.inclineLimittingLimit);
        const double startRad = node->getUserData().slopeDirectionAtan2 - limitRad;
        const double width = 2 * limitRad;
        assert(width >= 0);//this happens if the travmap was generated with a different maxSlope than config.maxSlope
        const base::AngleSegment segment(base::Angle::fromRad(startRad), width);
        const base::AngleSegment segmentMirrored(base::Angle::fromRad(startRad - M_PI), width);
        node->getUserData().allowedOrientations.emplace_back(base::Angle::fromRad(startRad), width);
        node->getUserData().allowedOrientations.emplace_back(base::Angle::fromRad(startRad - M_PI), width);
    }
    
    return true;
}



double TraversabilityGenerator3d::interpolate(double x, double x0, double y0, double x1, double y1) const
{
    //linear interpolation
    return y0 + (x - x0) * (y1 - y0)/(x1-x0);
}


double TraversabilityGenerator3d::computeSlope(const Eigen::Hyperplane< double, int(3) >& plane) const
{
    const Eigen::Vector3d zNormal(Eigen::Vector3d::UnitZ());
    Eigen::Vector3d planeNormal = plane.normal();
    planeNormal.normalize(); //just in case
    return acos(planeNormal.dot(zNormal));
}

Eigen::Vector3d TraversabilityGenerator3d::computeSlopeDirection(const Eigen::Hyperplane< double, int(3) >& plane) const
{
    /** The vector of maximum slope on a plane is the projection of (0,0,1) onto the plane.
     *  (0,0,1) is the steepest vector possible in the global frame, thus by projecting it onto
     *  the plane we get the steepest vector possible on that plane.
     */
    const Eigen::Vector3d zNormal(Eigen::Vector3d::UnitZ());
    const Eigen::Vector3d planeNormal(plane.normal().normalized());
    const Eigen::Vector3d projection = zNormal - zNormal.dot(planeNormal) * planeNormal;
    return projection;
}


bool TraversabilityGenerator3d::checkForUnknown(TravGenNode* node)
{
    //check direct neighborhood for missing connected patches. If 
    //patches are missing, this patch is unknown
    size_t missingNeighbors = 0;
    using maps::grid::Index;
    const Index& index = node->getIndex();
    for(int x = -1; x <= 1; ++x)
    {
        for(int y = -1; y <= 1; ++y)
        {
            const Index neighborIndex(index.x() + x, index.y() + y);
            const TravGenNode* neighbor = node->getConnectedNode(neighborIndex);
            if(neighbor == nullptr)
            {
                ++missingNeighbors;
            }
            else
            {
                //Draw connection to found neighbor
//                 COMPLEX_DRAWING(
//                     Eigen::Vector3d neighborPos(neighbor->getIndex().x() * config.gridResolution, neighbor->getIndex().y() * config.gridResolution, neighbor->getHeight());
//                     neighborPos.x() += config.gridResolution / 2.0;
//                     neighborPos.y() += config.gridResolution / 2.0;
//                     neighborPos = getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * neighborPos;
//                     neighborPos.z() += 0.06;
//                     Eigen::Vector3d pos(node->getIndex().x() * config.gridResolution, node->getIndex().y() * config.gridResolution, node->getHeight());
//                     pos.x() += config.gridResolution / 2.0;
//                     pos.y() += config.gridResolution / 2.0;
//                     pos = getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
//                     pos.z() += 0.06;
//                     DRAW_LINE("neighbor connections", pos, neighborPos, vizkit3dDebugDrawings::Color::magenta);
//                 );
            }
        }
    }
    
//     COMPLEX_DRAWING(
//         Eigen::Vector3d pos(node->getIndex().x() * config.gridResolution, node->getIndex().y() * config.gridResolution, node->getHeight());
//         pos.x() += config.gridResolution / 2.0;
//         pos.y() += config.gridResolution / 2.0;
//         pos = getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
//         pos.z() += 0.06;
//         DRAW_TEXT("missingNeighboursCount", pos, std::to_string(missingNeighbors), 0.02, vizkit3dDebugDrawings::Color::magenta);
//     );
    
    // >1 because the loop iterates over node as well, which is obviously unknown
    return missingNeighbors > 1;
}

bool TraversabilityGenerator3d::checkForObstacles(TravGenNode *node)
{
    const Eigen::Hyperplane<double, 3> &plane(node->getUserData().plane);
    const Eigen::Vector3d planeNormal(plane.normal().normalized());
    double slope = node->getUserData().slope;
    
    if(slope > config.maxSlope)
    {
        return false;
    }
    
    //filter out steps that are too steep for the robot
    Eigen::Vector3d nodePos;
    if(!trMap.fromGrid(node->getIndex(), nodePos))
        throw std::runtime_error("TraversabilityGenerator3d: Internal error node out of grid");
    nodePos.z() += node->getHeight();
    
    Eigen::Vector3d min(-config.robotSizeX / 2.0, -config.robotSizeX / 2.0, 0);
    Eigen::Vector3d max(-min);
    max.z() = config.robotHeight;
    
    min += nodePos;
    max += nodePos;
    
    View area = mlsGrid->intersectCuboid(Eigen::AlignedBox3d(min, max));
    
    for(size_t y = 0; y < area.getNumCells().y(); y++)
    {
        for(size_t x = 0; x < area.getNumCells().x(); x++)
        {
            Eigen::Vector3d pos;
            if(!area.fromGrid(Index(x,y), pos))
            {
                throw std::runtime_error("WTF");
            }

            for(const SurfacePatchBase *p : area.at(x, y))
            {
                pos.z() = p->getTop();
                float dist = plane.absDistance(pos);
                //bounding box already checks height of robot
                if(dist > config.maxStepHeight)
                {
                    return false;
                }
            }
        }
    }
    
    return true;
}

void TraversabilityGenerator3d::setConfig(const TraversabilityConfig &config)
{
    UGV_DEBUG(
        debugData.setTravConfig(config);
    )
    this->config = config;
}

void TraversabilityGenerator3d::expandAll(const Eigen::Vector3d& startPosWorld)
{
    TravGenNode *startNode = generateStartNode(startPosWorld);

    expandAll(startNode);
}

void TraversabilityGenerator3d::expandAll(TravGenNode* startNode)
{
    if(!startNode)
        return;

    std::deque<TravGenNode *> candidates;
    candidates.push_back(startNode);
    
    int cnd = 0;
    
    while(!candidates.empty())
    {
        TravGenNode *node = candidates.front();
        candidates.pop_front();

        //check if the node was evaluated before somehow
        if(node->isExpanded())
            continue;
        
        cnd++;
        
        if((cnd % 1000) == 0)
        {
            std::cout << "Expanded " << cnd << " nodes" << std::endl;
        }
        
        if(!expandNode(node))
        {
            continue;
        }

        for(auto *n : node->getConnections())
        {
            if(!n->isExpanded())
                candidates.push_back(static_cast<TravGenNode *>(n));
        }
    }
    
    std::cout << "Expanded " << cnd << " nodes" << std::endl;
}

void TraversabilityGenerator3d::addInitialPatchToMLS()
{
    std::cout << "Adding Initial Patch" << std::endl;
    const Vector2d res = mlsGrid->getResolution();
    
//         const double sizeHalfX = config.robotSizeX / 2.0;
//         const double sizeHalfY = config.robotSizeY / 2.0;

    const double sizeHalfX = patchRadius;
    const double sizeHalfY = patchRadius;
    
    //we oversample by factor 2 to account for aliasing
    for(double x = -sizeHalfX; x <= sizeHalfX; x += res.x() / 2.0)
    {
        for(double y = -sizeHalfY; y <= sizeHalfY; y += res.y() / 2.0)
        {
            if(Vector2d(x,y).norm() > patchRadius)
                continue;
            
            Vector3d pos(x, y, 0);
            Vector3d posMLS = initialPatch2Mls * pos;
            
            Index idx;
            if(!mlsGrid->toGrid(posMLS, idx))
            {
                std::cout << "Something is wrong, inital position is outside of MLS !" << std::endl;
                continue;
            }
            
            auto &ll = mlsGrid->at(idx);
            
            bool hasPatch = false;
            for(const SurfacePatchBase &p: ll)
            {
                if(p.isCovered(posMLS.z(), 0.05))
                {
                    hasPatch = true;
                    std::cout << "Found Patch at " << posMLS.transpose() << std::endl;
                    break;
                }
            }
            
            if(hasPatch)
                continue;
            
            SurfacePatchBase newPatch(posMLS.z());
            std::cout << "Adding Patch at " << posMLS.transpose() << std::endl;
            
            ll.insert(newPatch);
        }
    }

}

void TraversabilityGenerator3d::setMLSGrid(boost::shared_ptr< MLGrid >& grid)
{
    mlsGrid = grid;
    
    if(addInitialPatch)
    {
        addInitialPatchToMLS();
    }
    
    std::cout << "Grid has size " << grid->getSize().transpose() << " resolution " << grid->getResolution().transpose() << std::endl;
    std::cout << "Internal resolution is " << trMap.getResolution().transpose() << std::endl;
    Vector2d newSize = grid->getSize().array() / trMap.getResolution().array();
    std::cout << "MLS was set " << grid->getResolution().transpose() << " " << mlsGrid->getResolution().transpose() << std::endl;

    std::cout << "New Size is " << newSize.transpose() << std::endl;
    
    trMap.extend(Vector2ui(newSize.x(), newSize.y()));
    
    trMap.getLocalFrame() = mlsGrid->getLocalFrame();
    
    clearTrMap();
}

void TraversabilityGenerator3d::clearTrMap()
{
    for(LevelList<TravGenNode *> &l : trMap)
    {
        for(TravGenNode *n : l)
        {
            delete n;
        }
        
        l.clear();
    }
}

TravGenNode* TraversabilityGenerator3d::generateStartNode(const Eigen::Vector3d& startPosWorld)
{
    Index idx;
    if(!trMap.toGrid(startPosWorld, idx))
    {
        std::cout << "Start position outside of map !" << std::endl;
        return nullptr;
    }

    //check if not already exists...
    auto candidates = trMap.at(idx);
    for(TravGenNode *node : candidates)
    {
        if(fabs(node->getHeight() - startPosWorld.z()) < config.maxStepHeight)
        {
            std::cout << "TraversabilityGenerator3d::generateStartNode: Using existing node " << std::endl;
            return node;
        }
    }

    
    TravGenNode *startNode = new TravGenNode(startPosWorld.z(), idx);
    startNode->getUserData().id = currentNodeId++;
    trMap.at(idx).insert(startNode);

    return startNode;
}


bool TraversabilityGenerator3d::expandNode(TravGenNode * node)
{
    Eigen::Vector3d nodePos;
    if(!trMap.fromGrid(node->getIndex(), nodePos))
        throw std::runtime_error("TraversabilityGenerator3d: Internal error node out of grid");
    
    nodePos.z() += node->getHeight();
    
    //get all surfaces in a cube of robotwidth and stepheight
    Eigen::Vector3d min(-config.robotSizeX / 2.0, -config.robotSizeX / 2.0, -config.maxStepHeight);
    Eigen::Vector3d max(-min);
    
    min += nodePos;
    max += nodePos;
    
    View intersections = mlsGrid->intersectCuboid(Eigen::AlignedBox3d(min, max));
    
    node->setExpanded();
    
    //NOTE computePlane must be done before checkForObstacles !
    if(!computePlaneRansac(*node, intersections))
    {
        node->setType(TraversabilityNodeBase::UNKNOWN);
        return false;
    }

    if(!checkForObstacles(node))
    {
        node->setType(TraversabilityNodeBase::OBSTACLE);
        return false;
    }
    
    if(!computeAllowedOrientations(node))
    {
        node->setType(TraversabilityNodeBase::OBSTACLE);
        return false;
    }

    //add sourounding 
    addConnectedPatches(node);

    //FIXME rename to checkForFrontier
    if(checkForUnknown(node))
    {
        node->setType(TraversabilityNodeBase::FRONTIER);
    }
    else
    {
        node->setType(TraversabilityNodeBase::TRAVERSABLE);
    }
    
    return true;
}

void TraversabilityGenerator3d::addConnectedPatches(TravGenNode *  node)
{  
    static std::vector<Eigen::Vector2i> surounding = {
        Eigen::Vector2i(1, 1),
        Eigen::Vector2i(1, 0),
        Eigen::Vector2i(1, -1),
        Eigen::Vector2i(0, 1),
        Eigen::Vector2i(0, -1),
        Eigen::Vector2i(-1, 1),
        Eigen::Vector2i(-1, 0),
        Eigen::Vector2i(-1, -1)};

    double curHeight = node->getHeight();
    for(const Eigen::Vector2i &motion : surounding)
    {
        const Eigen::Vector2i &idxS(motion);
        const Index idx(node->getIndex().x() + idxS.x(), node->getIndex().y() + idxS.y());
        
        if(!trMap.inGrid(idx))
        {
            continue;
        }

        //compute height of cell in respect to plane
        Vector3d patchPosPlane(idxS.x() * trMap.getResolution().x(), idxS.y() * trMap.getResolution().y(), 0);
        Eigen::ParametrizedLine<double, 3> line(patchPosPlane, Eigen::Vector3d::UnitZ());
        Vector3d newPos =  line.intersectionPoint(node->getUserData().plane);
        
        if((patchPosPlane.head(2) - newPos.head(2)).norm() > 0.001)
            throw std::runtime_error("TraversabilityGenerator3d: Error, adjustement height calculation is weird");

        //The new patch is not reachable from the current patch
        if(fabs(newPos.z() - curHeight) > config.maxStepHeight)
            continue;
        
        curHeight = newPos.z();
        
        TravGenNode *toAdd = nullptr;

        if(!newPos.allFinite())
        {
            std::cout << "newPos contains inf" << std::endl;
            continue;
        }
        
        //check if we got an existing node
        for(TravGenNode *snode : trMap.at(idx))
        {
            const double searchHeight = snode->getHeight();
            if((searchHeight - config.maxStepHeight) <= curHeight && (searchHeight + config.maxStepHeight) >= curHeight)
            {
                //found a connectable node
                toAdd = snode;                
                break;
            }
            
            if(searchHeight > curHeight)
                break;
        }
        
        //no existing node exists at that location.
        //create a new one if a corresponding node in the mls exists
        if(!toAdd)
        {
            maps::grid::Vector3d globalPos;
            trMap.fromGrid(idx, globalPos);
            Index mlsIdx;
            if(mlsGrid->toGrid(globalPos, mlsIdx))
            {
                const auto& patches = mlsGrid->at(mlsIdx);
                for(const SurfacePatchBase& patch : patches)
                {
                    const double height = patch.getTop();//FIXME is getTop correct?
                    if((height - config.maxStepHeight) <= curHeight && (height + config.maxStepHeight) >= curHeight)
                    {
                        //there is a neighboring patch in the mls that has a reachable hight
                        toAdd = new TravGenNode(height, idx);
                        toAdd->getUserData().id = currentNodeId++;
                        trMap.at(idx).insert(toAdd);
//                         COMPLEX_DRAWING(
//                             maps::grid::Vector3d pos(globalPos);
//                             trMap.fromGrid(idx, globalPos);
//                             pos.z() = height;
//                             DRAW_RING("neighbor patches", pos, mlsGrid->getResolution().x() / 2.0, 0.4, 0.01, vizkit3dDebugDrawings::Color::blue);
//                         );
                        break;
                    }
                    if(height > curHeight)
                        break;
                }
            }
        }

        if(toAdd)
        {
            auto& connections = toAdd->getConnections();
            if(std::find(connections.begin(), connections.end(), node) == connections.end())
                toAdd->addConnection(node);
            
            auto& connections2 = node->getConnections();
            if(std::find(connections2.begin(), connections2.end(), toAdd) == connections2.end())
                node->addConnection(toAdd);
        }
    }
}

TraversabilityMap3d< TraversabilityNodeBase *> TraversabilityGenerator3d::getTraversabilityBaseMap() const
{
    maps::grid::TraversabilityMap3d<maps::grid::TraversabilityNodeBase *> trBaseMap(trMap.getNumCells(), trMap.getResolution(), trMap.getLocalMapData());
    
    for(size_t y = 0 ; y < trMap.getNumCells().y(); y++)
    {
        for(size_t x = 0 ; x < trMap.getNumCells().x(); x++)
        {
            Index idx(x, y);
            for(auto &p : trMap.at(idx))
            {
                trBaseMap.at(idx).insert(p);
            }
        }
    }
    
    return trBaseMap;
}


//Adapted from: https://rosettacode.org/wiki/Dijkstra%27s_algorithm#C.2B.2B
void TraversabilityGenerator3d::dijkstraComputeCost(const TravGenNode* source,
                          std::vector<double> &outDistances, const double maxDist) const 
{
    using namespace maps::grid;
    
    outDistances.clear();
    outDistances.resize(getNumNodes(), maxDist);
    
    const int sourceId = source->getUserData().id;
    outDistances[sourceId] = 0;
    
    std::set<std::pair<double, const TravGenNode*>> vertexQ;
    vertexQ.insert(std::make_pair(outDistances[sourceId], source));
    
    UGV_DEBUG(
        debugData.clearHeuristic();
    )
    
    while (!vertexQ.empty()) 
    {
        double dist = vertexQ.begin()->first;
        const TravGenNode* u = vertexQ.begin()->second;
        
        vertexQ.erase(vertexQ.begin());
        
        const Eigen::Vector3d uPos(u->getIndex().x() * config.gridResolution,
                                   u->getIndex().y() * config.gridResolution,
                                   u->getHeight());
        
        // Visit each edge exiting u
        for(TraversabilityNodeBase *v : u->getConnections())
        {   
            //skip all non traversable nodes. They will retain the maximum cost.
            if(v->getType() != TraversabilityNodeBase::TRAVERSABLE && 
               v->getType() != TraversabilityNodeBase::FRONTIER)
                continue;
            
            TravGenNode* vCasted = static_cast<TravGenNode*>(v);
            const Eigen::Vector3d vPos(vCasted->getIndex().x() * config.gridResolution,
                                       vCasted->getIndex().y() * config.gridResolution,
                                       vCasted->getHeight());

            const double distance = getHeuristicDistance(vPos, uPos);
            double distance_through_u = dist + distance;
            const int vId = vCasted->getUserData().id;
            
            if (distance_through_u < outDistances[vId])
            {
                vertexQ.erase(std::make_pair(outDistances[vId], vCasted));
                outDistances[vId] = distance_through_u;
                vertexQ.insert(std::make_pair(outDistances[vId], vCasted));
                UGV_DEBUG(
                    debugData.addHeuristicCost(vCasted, outDistances[vId]); 
                )
            }
        }
    }
}

double TraversabilityGenerator3d::getHeuristicDistance(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const
{
    switch(config.heuristicType)
    {
        case HeuristicType::HEURISTIC_2D:
            return (a.topRows(2) - b.topRows(2)).norm();
        case HeuristicType::HEURISTIC_3D:
            return (a - b).norm();
        default:
            throw std::runtime_error("unknown heuristic type");
    }
}


}
