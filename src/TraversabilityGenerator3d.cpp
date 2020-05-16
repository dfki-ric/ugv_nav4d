#include "TraversabilityGenerator3d.hpp"
#include <numeric/PlaneFitting.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <vizkit3d_debug_drawings/DebugDrawing.hpp>
#include <vizkit3d_debug_drawings/DebugDrawingColors.hpp>

#include <deque>
using namespace maps::grid;

namespace ugv_nav4d
{

TraversabilityGenerator3d::TraversabilityGenerator3d(const TraversabilityConfig& config) : addInitialPatch(false), config(config)
{
    trMap.setResolution(Eigen::Vector2d(config.gridResolution, config.gridResolution));

}

TraversabilityGenerator3d::~TraversabilityGenerator3d()
{
    clearTrMap();
}

void TraversabilityGenerator3d::setInitialPatch(const Eigen::Affine3d& ground2Mls, double patchRadius)
{
    initialPatch2Mls = ground2Mls;
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


bool TraversabilityGenerator3d::computePlaneRansac(TravGenNode& node)
{
    Eigen::Vector3d nodePos;
    if(!trMap.fromGrid(node.getIndex(), nodePos, node.getHeight()))
    {
        std::cout << "TraversabilityGenerator3d: Internal error node out of grid" << std::endl;
        return false;
    }

    const double growSize = std::min(config.robotSizeX, config.robotSizeY) / 2.0;

    //get all surfaces in a cube of robotwidth and stepheight
    Eigen::Vector3d min(-growSize, -growSize, -config.maxStepHeight);
    Eigen::Vector3d max(-min);
    
    min += nodePos;
    max += nodePos;
    const Eigen::AlignedBox3d searchArea(min, max);
    View area = mlsGrid->intersectCuboid(searchArea);
    
    
    typedef pcl::PointXYZ PointT;
    
    pcl::PointCloud<PointT>::Ptr points(new pcl::PointCloud<PointT>());
    
    Eigen::Vector2d sizeHalf(area.getSize() / 2.0);
    
    const Eigen::Vector2d& res = mlsGrid->getResolution();
    
    
    const int patchCntTotal = area.getNumCells().y() * area.getNumCells().x(); //FIXME only works if there is only one patch per cell
    int patchCnt = 0;
    for(size_t y = 0; y < area.getNumCells().y(); y++)
    {
        for(size_t x = 0; x < area.getNumCells().x(); x++)
        {
            Eigen::Vector2d pos = Eigen::Vector2d(x,y).cwiseProduct(res) - sizeHalf;
            
            bool hasPatch = false;
            for(const MLGrid::PatchType *p : area.at(x, y))
            {
                PointT pclP(pos.x(), pos.y(), p->getMean());
                points->push_back(pclP);
                hasPatch = true;
            }
            
            if(hasPatch)
                patchCnt++;
        }
    }

    
    //if less than 5 planes -> hole
    //TODO where to implement ? here or in check obstacles ?
    if(patchCnt < 5)
    {
        //ransac will not produce a result below 5 points
//         std::cout << "ransac fail: patchCnt < 5" << std::endl;
        return false;
    }
    
    //filter out to sparse areas
    if(patchCnt < patchCntTotal * config.minTraversablePercentage)
    {
//         std::cout << "TraversabilityGenerator3d::computePlaneRansac : not enough known patches : patchCnt " << patchCnt << " patchCntTotal " << patchCntTotal << " val " << patchCntTotal * config.minTraversablePercentage << std::endl;
        return false;
    }

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
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
    seg.segment (inliers, coefficients);
    if (inliers.indices.size () <= 5)
    {
//         std::cout << "ransac fail: inliers->indices.size () <= 5" << std::endl;
        return false;
    }


    Eigen::Vector3d normal(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
    normal.normalize();
    double distToOrigin = coefficients.values[3];
    
    node.getUserData().plane = Eigen::Hyperplane<double, 3>(normal, distToOrigin);
    
    //adjust height of patch
    Eigen::ParametrizedLine<double, 3> line(Vector3d::Zero(), Eigen::Vector3d::UnitZ());
    Vector3d newPos =  line.intersectionPoint(node.getUserData().plane);
    
    if(newPos.x() > 0.0001 || newPos.y() > 0.0001)
    {
        std::cout << "TraversabilityGenerator3d: Error, adjustement height calculation is weird" << std::endl;
        return false;
    }

    if(newPos.allFinite())
    {
        node.setHeight(newPos.z());
    }    
    
    const Eigen::Vector3d slopeDir = computeSlopeDirection(node.getUserData().plane);
    node.getUserData().slope = computeSlope(node.getUserData().plane);
    node.getUserData().slopeDirection = slopeDir;
    node.getUserData().slopeDirectionAtan2 = std::atan2(slopeDir.y(), slopeDir.x());
    
//     COMPLEX_DRAWING(
//         Eigen::Vector3d pos(node.getIndex().x() * config.gridResolution, node.getIndex().y() * config.gridResolution, node.getHeight());
//         pos = getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
//         pos.z() += 0.06;
//         DRAW_TEXT("slope", pos, std::to_string(node.getUserData().slope), 0.01, V3DD::Color::red);
//     );
        
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
        //only allow certail orientations because we are above the slope limit
        const double limitRad = interpolate(node->getUserData().slope, config.inclineLimittingMinSlope,
                                         M_PI_2, config.maxSlope, config.inclineLimittingLimit);
        const double startRad = node->getUserData().slopeDirectionAtan2 - limitRad;
        const double width = 2 * limitRad;
        assert(width >= 0);//this happens if the travmap was generated with a different maxSlope than config.maxSlope
//         const base::AngleSegment segment(base::Angle::fromRad(startRad), width);
//         const base::AngleSegment segmentMirrored(base::Angle::fromRad(startRad - M_PI), width);
        
        //add forward allowed angles
        node->getUserData().allowedOrientations.emplace_back(base::Angle::fromRad(startRad), width);
        
//         V3DD::COMPLEX_DRAWING([&]()
//         {
//             Eigen::Vector3d patchPos(node->getIndex().x() * config.gridResolution, node->getIndex().y() * config.gridResolution, node->getHeight());
//             patchPos.x() += config.gridResolution / 2.0;
//             patchPos.y() += config.gridResolution / 2.0;
//             patchPos = getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * patchPos;
//             patchPos.z() += 0.06;
// 
//             Eigen::AngleAxisd rot1(node->getUserData().allowedOrientations.back().getStart().getRad(), Eigen::Vector3d::UnitZ());
//             Eigen::AngleAxisd rot2(node->getUserData().allowedOrientations.back().getEnd().getRad(), Eigen::Vector3d::UnitZ());
//             
//             Eigen::Vector3d end1 = rot1 * Eigen::Vector3d(0.1, 0, 0);
//             Eigen::Vector3d end2 = rot2 * Eigen::Vector3d(0.1, 0, 0);
//                     
//             V3DD::DRAW_LINE("ugv_nav4d_allowedAngles", patchPos, patchPos + end1, V3D:::Color::magenta);
//             V3DD::DRAW_LINE("ugv_nav4d_allowedAngles", patchPos, patchPos + end2, V3DD::Color::magenta);
//         });
        
        //add backward allowed angles
        if(config.allowForwardDownhill)
        {
            node->getUserData().allowedOrientations.emplace_back(base::Angle::fromRad(startRad - M_PI), width);
            
            V3DD::COMPLEX_DRAWING([&]()
            {
                Eigen::Vector3d patchPos(node->getIndex().x() * config.gridResolution, node->getIndex().y() * config.gridResolution, node->getHeight());
                patchPos.x() += config.gridResolution / 2.0;
                patchPos.y() += config.gridResolution / 2.0;
                patchPos = getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * patchPos;
                patchPos.z() += 0.06;

                Eigen::AngleAxisd rot1(node->getUserData().allowedOrientations.back().getStart().getRad(), Eigen::Vector3d::UnitZ());
                Eigen::AngleAxisd rot2(node->getUserData().allowedOrientations.back().getEnd().getRad(), Eigen::Vector3d::UnitZ());
                
                Eigen::Vector3d end1 = rot1 * Eigen::Vector3d(0.1, 0, 0);
                Eigen::Vector3d end2 = rot2 * Eigen::Vector3d(0.1, 0, 0);
                        
                V3DD::DRAW_LINE("ugv_nav4d_allowedAngles", patchPos, patchPos + end1, V3DD::Color::cyan);
                V3DD::DRAW_LINE("ugv_nav4d_allowedAngles", patchPos, patchPos + end2, V3DD::Color::cyan);
            });
            
        }
        
    }
    
    return true;
}



double TraversabilityGenerator3d::interpolate(double x, double x0, double y0, double x1, double y1)
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


bool TraversabilityGenerator3d::checkForFrontier(const TravGenNode* node)
{
    //check direct neighborhood for missing connected patches. If 
    //patches are missing, this patch is unknown
    using maps::grid::Index;
    const Index& index = node->getIndex();
    for(int x = -1; x <= 1; ++x)
    {
        for(int y = -1; y <= 1; ++y)
        {
            if(x==0 && y==0) continue;
            const Index neighborIndex(index.x() + x, index.y() + y);
            const TravGenNode* neighbor = node->getConnectedNode(neighborIndex);
            if(neighbor == nullptr || neighbor->getType() == TraversabilityNodeBase::UNKNOWN)
            {
                return true;
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
//                     DRAW_LINE("neighbor connections", pos, neighborPos, V3DD::Color::magenta);
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
//         DRAW_TEXT("missingNeighboursCount", pos, std::to_string(missingNeighbors), 0.02, V3DD::Color::magenta);
//     );
    return false;
}

bool TraversabilityGenerator3d::checkStepHeight(TravGenNode *node)
{
    
    /** What this method does:
     * Check if any of the patches around @p node that the robot might stand on is higher than stepHeight.
     * I.e. if any of the patches is so high that it would be inside the robots body.
     */ 
    
    const double slope = node->getUserData().slope;
    
    if(slope > config.maxSlope)
    {
        return false;
    }
    

    Eigen::Vector3d nodePos;
    if(!trMap.fromGrid(node->getIndex(), nodePos))
    {
        std::cout << "TraversabilityGenerator3d: Internal error node out of grid" << std::endl;
        return false;
    }
    nodePos.z() += node->getHeight();

    const double smallerRobotSize = std::min(config.robotSizeX, config.robotSizeY);
    const double growSize = smallerRobotSize / 2.0 + 1e-5;
    
    Eigen::Vector3d min(-growSize, -growSize, 0);
    Eigen::Vector3d max(-min);
    max.z() = config.robotHeight;
    
    min += nodePos;
    max += nodePos;
    
    const Eigen::AlignedBox3d limitBox(min, max);
    View area = mlsGrid->intersectCuboid(limitBox);
    
    
    
//     V3DD::COMPLEX_DRAWING([&]
//     {   
//         static int i = 0;
//         ++i;
//         if((i % 30) == 0)
//             V3DD::DRAW_AABB("ugv_nav4d_trav_obstacle_check_box", limitBox, V3DD::Color::red);
//     });
    
    
    const Eigen::Hyperplane<double, 3> &plane(node->getUserData().plane);
    

    Index minIdx;
    Index maxIdx;
    
    
    if(!mlsGrid->toGrid(limitBox.min(), minIdx))
    {
        //when the robot bounding box leaves the map this patch cannot be traversable
        return false;
    }
    if(!mlsGrid->toGrid(limitBox.max(), maxIdx))
    {
        //when the robot bounding box leaves the map this patch cannot be traversable
        return false;
    }
 
    Index curIndex = minIdx;
    Index areaSize(maxIdx-minIdx);
    
    //the robot size has been set to a value smaller than one cell. Thus we cannot check anything.
    if(area.getNumCells().y() <= 0 || area.getNumCells().x() <= 0)
        return true;
    
    //intersectCuboid returns an area that is one patch bigger than requested. I.e.
    //it interpretes both min and max as inclusive. However, we consider max to be exclusive and
    //thus reduce the cell count by one
    for(size_t y = 0; y < area.getNumCells().y() - 1; y++, curIndex.y() += 1)
    {
        curIndex.x() = minIdx.x();
        for(size_t x = 0; x < area.getNumCells().x() - 1; x++, curIndex.x() += 1)
        {
            Eigen::Vector3d pos;
            if(!area.fromGrid(Index(x,y), pos))
            {
                std::cout << "this should never happen" << std::endl;
            }

            for(const SurfacePatch<MLSConfig::KALMAN> *p : area.at(x, y))
            {
                pos.z() = p->getMean();
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

void TraversabilityGenerator3d::growNodes()
{
    const double growRadiusSquared = std::pow(std::sqrt(config.robotSizeX * config.robotSizeX + config.robotSizeY * config.robotSizeY) / 2.0, 2);
    
    for(TravGenNode *n : growList)
    {
        Eigen::Vector3d nodePos = n ->getPosition(trMap);
        
        n->eachConnectedNode([&](maps::grid::TraversabilityNodeBase *neighbor, bool &expandNode, bool &stop)
        {
            if((neighbor->getPosition(trMap) - nodePos).squaredNorm() > growRadiusSquared)
            {
                //node out of radius, return
                return;
            }
            
            expandNode = true;
            
            switch(neighbor->getType())
            {
                case TraversabilityNodeBase::FRONTIER:
                    if(n->getType() == TraversabilityNodeBase::OBSTACLE)
                        neighbor->setType(n->getType());
                    break;
                case TraversabilityNodeBase::TRAVERSABLE:
                    neighbor->setType(n->getType());
                    break;
                default:
                    break;
            }
            
            if(neighbor->getType() == TraversabilityNodeBase::TRAVERSABLE)
            {
            }
        });
    }
    
    growList.clear();
}

void TraversabilityGenerator3d::setConfig(const TraversabilityConfig &config)
{
    this->config = config;
}

void TraversabilityGenerator3d::expandAll(const Eigen::Vector3d& startPos)
{
    TravGenNode *startNode = generateStartNode(startPos);

    expandAll(startNode);
}

void TraversabilityGenerator3d::expandAll(const std::vector<Eigen::Vector3d>& positions)
{
    for(const Eigen::Vector3d& pos : positions)
    {
        expandAll(pos);
    }
}


void TraversabilityGenerator3d::expandAll(const Eigen::Vector3d& startPos, const double expandDist)
{
    TravGenNode *startNode = generateStartNode(startPos);
    expandAll(startNode, expandDist);    
}


void TraversabilityGenerator3d::expandAll(TravGenNode* startNode)
{
    expandAll(startNode, -1.0);
}
void TraversabilityGenerator3d::expandAll(TravGenNode* startNode, const double expandDist)
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
            {
                if(expandDist > 0)
                {
                    const double dist = (startNode->getPosition(trMap) - n->getPosition(trMap)).norm();
                    if(dist <= expandDist)
                        candidates.push_back(static_cast<TravGenNode *>(n));
                }
                else
                {
                    candidates.push_back(static_cast<TravGenNode *>(n));
                }
            }
        }
    }
    
    //grow frontier nodes
    growNodes();
    
    std::cout << "Expanded " << cnd << " nodes" << std::endl;
}

void TraversabilityGenerator3d::addInitialPatchToMLS()
{
    if(patchRadius == 0)
        return;
    
    
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
                std::cout << "Something is wrong, initial position is outside of MLS !" << std::endl;
                continue;
            }
            
            auto &ll = mlsGrid->at(idx);
            
            bool hasPatch = false;
            for(const MLGrid::PatchType &p: ll)
            {
                if(p.isCovered(posMLS.z(), 0.05))
                {
                    hasPatch = true;
//                     std::cout << "Found Patch at " << posMLS.transpose() << std::endl;
                    break;
                }
            }
            
            if(hasPatch)
                continue;
                        
            MLGrid::PatchType newPatch(posMLS.z(), config.initialPatchVariance);
//             std::cout << "Adding Patch at " << posMLS.transpose() << std::endl;
            
            ll.insert(newPatch);
        }
    }

}

void TraversabilityGenerator3d::setMLSGrid(std::shared_ptr< ugv_nav4d::TraversabilityGenerator3d::MLGrid >& grid)
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

TravGenNode* TraversabilityGenerator3d::generateStartNode(const Eigen::Vector3d& startPos)
{
    Index idx;
    if(!trMap.toGrid(startPos, idx))
    {
        std::cout << "TraversabilityGenerator3d::generateStartNode: Start position outside of map !" << std::endl;
        return nullptr;
    }

    //check if not already exists...
    TravGenNode *startNode = findMatchingTraversabilityPatchAt(idx, startPos.z());
    if(startNode)
    {
        std::cout << "TraversabilityGenerator3d::generateStartNode: Using existing node " << std::endl;
        return startNode;
    }

    startNode = createTraversabilityPatchAt(idx, startPos.z());
    if(!startNode)
    {
        std::cout << "TraversabilityGenerator3d::generateStartNode: Could not create travNode for given start position, no matching / not enough MLS patches" << std::endl;
        return startNode;
    }

    if(startNode->isExpanded() && startNode->getType() != TraversabilityNodeBase::TRAVERSABLE)
    {
        std::cout << "TraversabilityGenerator3d::generateStartNode: Position is on unknow patch ! " << std::endl;
        std::cout << "type is: " << startNode->getType() << std::endl;
    }
    
    return startNode;
}


bool TraversabilityGenerator3d::expandNode(TravGenNode * node)
{
    node->setExpanded();

    if(node->getType() == TraversabilityNodeBase::UNKNOWN
        || node->getType() == TraversabilityNodeBase::OBSTACLE)
    {
        return false;
    }

    if(!checkStepHeight(node))
    {
        node->setType(TraversabilityNodeBase::OBSTACLE);
        return false;
    }
    
    if(config.enableInclineLimitting)
    {
        if(!computeAllowedOrientations(node))
        {
            node->setType(TraversabilityNodeBase::OBSTACLE);
            return false;
        }
    }

    //add surrounding
    addConnectedPatches(node);

    if(checkForFrontier(node))
    {
        node->setType(TraversabilityNodeBase::FRONTIER);
        growList.push_back(node);
        return false;
    }

    node->setType(TraversabilityNodeBase::TRAVERSABLE);
    
    return true;
}

TravGenNode *TraversabilityGenerator3d::createTraversabilityPatchAt(maps::grid::Index idx, const double curHeight)
{
    TravGenNode *ret = nullptr;
    
    maps::grid::Vector3d globalPos;
    trMap.fromGrid(idx, globalPos);
    Index mlsIdx;
    if(!mlsGrid->toGrid(globalPos, mlsIdx))
    {
        return nullptr;
    }
    
    const auto& patches = mlsGrid->at(mlsIdx);
    
    std::vector<double> candidates;
    
    for(const SurfacePatch<MLSConfig::KALMAN>& patch : patches)
    {
        //We use top, as we drive on the surface
        const double height = patch.getMean();

        if((height - config.maxStepHeight) <= curHeight && (height + config.maxStepHeight) >= curHeight)
        {
            candidates.push_back(height);
        }
        if(height > (curHeight + config.maxStepHeight))
            break;
    }
    
    //Also add the interpolated height, to fill in small holes
    //if there is no support, the ransac will filter the node out
    candidates.push_back(curHeight);

    ret = new TravGenNode(0.0, idx);
    ret->getUserData().id = currentNodeId++;
    
    for(double height: candidates)
    {
        ret->setHeight(height);
        ret->setNotExpanded();
        ret->setType(TraversabilityNodeBase::UNSET);
        
        //there is a neighboring patch in the mls that has a reachable hight
        if(!computePlaneRansac(*ret))
        {
            if(mlsIdx.x() == 1 || mlsIdx.y() == 1) {
                ret->setType(TraversabilityNodeBase::OBSTACLE);
            } else {
                ret->setType(TraversabilityNodeBase::UNKNOWN);
            }
        }

        if((ret->getHeight() - config.maxStepHeight) <= curHeight && (ret->getHeight() + config.maxStepHeight) >= curHeight)
        {
            trMap.at(idx).insert(ret);
            return ret;
        }
        else
        {
            //rare border case, ransac correction moved patch out of reachable height
        }
        
//         COMPLEX_DRAWING(
//             maps::grid::Vector3d pos(globalPos);
//             trMap.fromGrid(idx, globalPos);
//             pos.z() = height;
//             DRAW_RING("neighbor patches", pos, mlsGrid->getResolution().x() / 2.0, 0.4, 0.01, V3DD::Color::blue);
//         );
    }
    
    ret->setHeight(curHeight);
    ret->setNotExpanded();
    ret->setType(TraversabilityNodeBase::OBSTACLE);
    trMap.at(idx).insert(ret);
    return ret;            
}

TravGenNode* TraversabilityGenerator3d::findMatchingTraversabilityPatchAt(Index idx, const double curHeight) const
{
    auto &trList(trMap.at(idx));

    //check if we got an existing node
    for(TravGenNode *snode : trList)
    {
        const double searchHeight = snode->getHeight();
        if((searchHeight - config.maxStepHeight) <= curHeight && (searchHeight + config.maxStepHeight) >= curHeight)
        {
            //found a connectable node
            return snode;
        }
        
        if(searchHeight > curHeight)
        {
            return nullptr;
        }
    }

    return nullptr;
}

void TraversabilityGenerator3d::addConnectedPatches(TravGenNode *  node)
{  
    static std::vector<Index> surounding = {
        Index(1, 1),
        Index(1, 0),
        Index(1, -1),
        Index(0, 1),
        Index(0, -1),
        Index(-1, 1),
        Index(-1, 0),
        Index(-1, -1)};

    double curHeight = node->getHeight();
    for(const Index &idxS : surounding)
    {
        const Index idx(node->getIndex() + idxS);
        
        if(!trMap.inGrid(idx))
        {
            continue;
        }

        //compute height of cell in respect to plane
        const Vector3d patchPosPlane(idxS.x() * trMap.getResolution().x(), idxS.y() * trMap.getResolution().y(), 0);
        const Eigen::ParametrizedLine<double, 3> line(patchPosPlane, Eigen::Vector3d::UnitZ());
        const Vector3d newPos =  line.intersectionPoint(node->getUserData().plane);
        
        //this happend at some point because of a bug somewhere else.
        //not sure if this can still happen.
        if((patchPosPlane.head(2) - newPos.head(2)).norm() > 0.001)
        {
            std::cout << "TraversabilityGenerator3d: FATAL ERROR, adjustment height calculation is weird" << std::endl;
            return;
        }

        const double localHeight = newPos.z();
        //The new patch is not reachable from the current patch
        if(fabs(localHeight - curHeight) > config.maxStepHeight)
        {
            V3DD::COMPLEX_DRAWING([&]()
            {
                maps::grid::Vector3d pos;
                trMap.fromGrid(node->getIndex(), pos, node->getHeight(), false);
                V3DD::DRAW_SPHERE("ugv_nav4d_expandFailStepHeight", pos, 0.05, V3DD::Color::carrot_orange);
            });
            
            continue;
        }
        
        
        TravGenNode *toAdd = nullptr;

        if(!newPos.allFinite())
        {
            std::cout << "newPos contains inf" << std::endl;
            continue;
        }
        
        //check if we got an existing node
        toAdd = findMatchingTraversabilityPatchAt(idx, localHeight);
        
        //no existing node exists at that location.
        //try to create a new one at the position
        if(!toAdd)
        {
            toAdd = createTraversabilityPatchAt(idx, localHeight);
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



}
