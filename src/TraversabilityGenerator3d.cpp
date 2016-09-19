#include "TraversabilityGenerator3d.hpp"
#include <numeric/PlaneFitting.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <deque>

using namespace maps::grid;

namespace ugv_nav4d
{

TraversabilityGenerator3d::TraversabilityGenerator3d(const TraversabilityConfig& config) : config(config)
{
    trMap.setResolution(Eigen::Vector2d(config.gridResolution, config.gridResolution));
}

TraversabilityGenerator3d::~TraversabilityGenerator3d()
{
    clearTrMap();
}

const maps::grid::TraversabilityMap3d< TraversabilityGenerator3d::Node * >& TraversabilityGenerator3d::getTraversabilityMap() const
{
    return trMap;
}

bool TraversabilityGenerator3d::computePlaneRansac(TraversabilityGenerator3d::Node& node, const View &area)
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

    //if less than 3 planes -> hole
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
    seg.setMaxIterations (5);
    seg.setDistanceThreshold (0.1);

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;


//     std::cout << "Num Points is " << points->size() << std::endl;
    
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (points);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate Ground Plane" << std::endl;
        return false;
    }


    Eigen::Vector3d normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    normal.normalize();
    double distToOrigin = coefficients->values[3];
    
    node.getUserData().plane = Eigen::Hyperplane<double, 3>(normal, distToOrigin);
    
    //adjust height of patch
    Eigen::ParametrizedLine<double, 3> line(Vector3d::Zero(), Eigen::Vector3d::UnitZ());
    Vector3d newPos =  line.intersectionPoint(node.getUserData().plane);
    
//     if(node.getHeight() > -0.2)
//         std::cout << "Corrected height from " << node.getHeight() << " to " << newPos.transpose() << std::endl;

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
//     if(node.getHeight() > -0.2)
//         std::cout << "Plane Computed for " << node.getIndex().transpose() << " : Normal " << plane.getNormal().transpose() << std::endl; 
    
    return true;
}


bool TraversabilityGenerator3d::computePlane( TraversabilityGenerator3d::Node & node, const View &area)
{
    //estimate the plane of the surfaces
    numeric::PlaneFitting<double> planeFit;
    
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
                pos.z() = p->getTop();
                planeFit.update(pos);
                
                patchCnt++;
            }
        }
    }

//     if(node.getHeight() > -0.2)
//         std::cout << "PatchCnt is " << patchCnt << std::endl;
    
    //if less than 3 planes -> hole
    //TODO where to implement ? here or in check obstacles ?
    if(patchCnt < 3)
    {
        return false;
    }
    numeric::PlaneFitting<double>::ResultNormal plane(planeFit.solveNormal());
    
    node.getUserData().plane = plane.getPlane();
    
    //adjust height of patch
    Eigen::ParametrizedLine<double, 3> line(Vector3d::Zero(), Eigen::Vector3d::UnitZ());
    Vector3d newPos =  line.intersectionPoint(plane.getPlane());
    
//     if(node.getHeight() > -0.2)
//         std::cout << "Corrected height from " << node.getHeight() << " to " << newPos.transpose() << std::endl;

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
//     if(node.getHeight() > -0.2)
//         std::cout << "Plane Computed for " << node.getIndex().transpose() << " : Normal " << plane.getNormal().transpose() << std::endl; 
    
    return true;
}

bool TraversabilityGenerator3d::checkForObstacles(const View& area, TraversabilityGenerator3d::Node *node)
{
    const Eigen::Hyperplane<double, 3> &plane(node->getUserData().plane);
    const Eigen::Vector3d planeNormal(plane.normal());
    double slope = acos(planeNormal.dot(Eigen::Vector3d::UnitZ()));
    
    if(slope > config.maxSlope)
    {
//         if(node->getHeight() > -0.2)
//             std::cout << "Slope to hight " << slope << " max is " << config.maxSlope << std::endl;
        return false;
    }
    
    
    for(size_t y = 0; y < area.getNumCells().y(); y++)
    {
        for(size_t x = 0; x < area.getNumCells().x(); x++)
        {
            Eigen::Vector3d pos;
            if(!area.fromGrid(Index(x,y), pos))
            {
                throw std::runtime_error("WTF");
            }

            //TODO the patches are ordered... Use this
            for(const SurfacePatchBase *p : area.at(x, y))
            {
                pos.z() = p->getTop();
                
                float dist = plane.signedDistance(pos);
                
                //TODO find out, of positive means above the plane....
                
                if(dist < config.robotHeight  && dist > config.maxStepHeight)
                {
//                     std::cout << "Found an obstcle dist is " << dist << std::endl;
                    return false;
                }
            }
        }
    }
    
    return true;
}

void TraversabilityGenerator3d::expandAll(const Eigen::Vector3d& startPosWorld)
{
    Node *startNode = generateStartNode(startPosWorld);

    expandAll(startNode);
}

void TraversabilityGenerator3d::expandAll( TraversabilityGenerator3d::Node* startNode)
{
    if(!startNode)
        return;

    std::deque<Node *> candidates;
    candidates.push_back(startNode);
    
    int cnd = 0;
    int cndLast = 0;
    
    while(!candidates.empty())
    {
        Node *node = candidates.front();
        candidates.pop_front();

        //check if the node was evaluated before somehow
        if(node->isExpanded())
            continue;
        
        cnd++;

        if(cnd - cndLast > 1000)
        {
//             std::cout  << "Expanded " << cnd << " Nodes " << std::endl;
            cndLast = cnd;
        }
        
        
//         if(cnd > 7000)
//             return;
        
        if(!expandNode(node))
        {
//             if(abs(node->getHeight()) > 1.0)
//                 std::cout << "Height is " << node->getHeight() << std::endl;
//             if(node->getHeight() > 5.0)
//                 return;
            continue;
        }

//         if(abs(node->getHeight()) > 1.0)
//             std::cout << "Height is " << node->getHeight() << std::endl;
//         if(node->getHeight() > 5.0)
//             return;

        for(auto *n : node->getConnections())
        {
            if(!n->isExpanded())
                candidates.push_back(static_cast<Node *>(n));
        }
    }
    
    std::cout << "Expanded " << cnd << " nodes" << std::endl;
}


void TraversabilityGenerator3d::setMLSGrid(boost::shared_ptr< MLGrid >& grid)
{
    mlsGrid = grid;
    
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
    for(LevelList<Node *> &l : trMap)
    {
        for(Node *n : l)
        {
            delete n;
        }
        
        l.clear();
    }
}

TraversabilityGenerator3d::Node* TraversabilityGenerator3d::generateStartNode(const Eigen::Vector3d& startPosWorld)
{
    Index idx;
    if(!trMap.toGrid(startPosWorld, idx))
    {
        std::cout << "Start position outside of map !" << std::endl;
        return nullptr;
    }

    //check if not already exists...
    auto candidates = trMap.at(idx);
    for(Node *node : candidates)
    {
        if(fabs(node->getHeight() - startPosWorld.z()) < config.maxStepHeight)
        {
            std::cout << "TraversabilityGenerator3d::generateStartNode: Using existing node " << std::endl;
            return node;
        }
    }

    
    Node *startNode = new Node(startPosWorld.z(), idx);
    trMap.at(idx).insert(startNode);

    return startNode;
}


bool TraversabilityGenerator3d::expandNode( TraversabilityGenerator3d::Node * node)
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
    
//     std::cout << "Node Pos " << nodePos.transpose() << std::endl;
//     std::cout << "Min Pos " << min.transpose() << std::endl;
//     std::cout << "Max Pos " << max.transpose() << std::endl;
//     
    View intersections = mlsGrid->intersectCuboid(Eigen::AlignedBox3d(min, max));

//     std::cout << "Size of Intersection " << intersections.getSize().transpose() << "  " << std::endl;
//     std::cout << "NumCells of Intersection " << intersections.getNumCells().transpose() << "  " << std::endl;
    
    node->setExpanded();
    
    //note, computePlane must be done before checkForObstacles !
    if(!computePlaneRansac(*node, intersections))
    {
//         std::cout << "Could not compute ransac" << std::endl;
        node->setType(TraversabilityNodeBase::UNKNOWN);
        return false;
    }

    if(!checkForObstacles(intersections, node))
    {
//         std::cout << "Node is obstacle" << std::endl;
        node->setType(TraversabilityNodeBase::OBSTACLE);
        return false;
    }
//     std::cout << "Patch is good" << std::endl;
    
    node->setType(TraversabilityNodeBase::TRAVERSABLE);
    
    //add sourounding 
    addConnectedPatches(node);
    
    return true;
}

bool TraversabilityGenerator3d::updateDistToStart(double newValue, TraversabilityNodeBase* node)
{
    const static double dist2 = sqrt(2);
    
    node->setDistToStart(newValue);
    
    for(TraversabilityNodeBase *conNode : node->getConnections())
    {
        double dist = 1;
        Eigen::Vector2i diff(Eigen::Vector2i(node->getIndex().x(), node->getIndex().y()) - Eigen::Vector2i(conNode->getIndex().x(), conNode->getIndex().y()));
        if(diff.squaredNorm() > 1)
        {
            dist = dist2;
        }
        
        double newDist = node->getDistToStart() + dist;
        
        trMap.maxDist = std::max(newDist, trMap.maxDist);
        
        if(conNode->getDistToStart() > newDist)
            updateDistToStart(newDist, conNode);
    }
    
    return false;
}


void TraversabilityGenerator3d::addConnectedPatches( TraversabilityGenerator3d::Node *  node)
{
    struct MotionWithDist
    {
        MotionWithDist(const Eigen::Vector2i &motion, double dir) : motion(motion), dist(dir)
        {
        };
        Eigen::Vector2i motion;
        double dist;
    };
    
    static std::vector<MotionWithDist> surounding = {
        {Eigen::Vector2i(1,1), sqrt(2.0)},
        {Eigen::Vector2i(1,0), 1.0},
        {Eigen::Vector2i(1,-1), sqrt(2.0)},
        {Eigen::Vector2i(0,1), 1.0},
        {Eigen::Vector2i(0,-1), 1.0},
        {Eigen::Vector2i(-1,1), sqrt(2.0)},
        {Eigen::Vector2i(-1,0), 1.0},
        {Eigen::Vector2i(-1,-1), sqrt(2.0)}
    };

//     std::cout << "Adding connections for " << node->getIndex().transpose() << std::endl<< std::endl;
    
    double curHeight = node->getHeight();
    for(const MotionWithDist &mwd : surounding)
    {
        const Eigen::Vector2i &idxS(mwd.motion);
        const Index idx(node->getIndex().x() + idxS.x(), node->getIndex().y() + idxS.y());
        
        
        if(!trMap.inGrid(idx))
        {
//             std::cout << "Out of Map" << std::endl;
            continue;
        }

        //compute height of cell in respect to plane
        Vector3d patchPosPlane(idxS.x() * trMap.getResolution().x(), idxS.y() * trMap.getResolution().y(), 0);
        Eigen::ParametrizedLine<double, 3> line(patchPosPlane, Eigen::Vector3d::UnitZ());
        Vector3d newPos =  line.intersectionPoint(node->getUserData().plane);
        
//         if(node->getHeight() > -0.2)
//         std::cout << "Corrected height from " << node->getHeight() << " to " << newPos.transpose() << std::endl;

        if((patchPosPlane.head(2) - newPos.head(2)).norm() > 0.001)
            throw std::runtime_error("TraversabilityGenerator3d: Error, adjustement height calculation is weird");

        curHeight = newPos.z();
        
        Node *toAdd = nullptr;
        
        //check if we got an existing node
        for(Node *snode : trMap.at(idx))
        {
//             std::cout << "Testing if " << snode->getHeight() << " is in reach of " << curHeight << std::endl;
            const double searchHeight = snode->getHeight();
            if((searchHeight - config.maxStepHeight) < curHeight && (searchHeight + config.maxStepHeight) > curHeight)
            {
                //found a connectable node !
                toAdd = snode;
                
//                 std::cout << "FOOOOOOOOOO" << std::endl;
                
//                 std::cout << "Found connectable node at " << idx.transpose() << std::endl;
                
                const double curDist = node->getDistToStart() + mwd.dist;
                
                if(toAdd->getDistToStart() > curDist)
                {
                    updateDistToStart(curDist, toAdd);
                }
                
                break;
            }
            
            if(searchHeight > curHeight)
                break;
        }
        
        if(!toAdd)
        {
//             std::cout << "No Node at " << idx.transpose() << " creating new one" << std::endl;
            toAdd = new Node(curHeight, idx);
            toAdd->setDistToStart(node->getDistToStart() + mwd.dist);
            trMap.maxDist = std::max(node->getDistToStart() + mwd.dist, trMap.maxDist);
            trMap.at(idx).insert(toAdd);
        }

        toAdd->addConnection(node);
        node->addConnection(toAdd);
    }
}

TraversabilityMap3d< TraversabilityNodeBase *> TraversabilityGenerator3d::getTraversabilityBaseMap() const
{
    maps::grid::TraversabilityMap3d<maps::grid::TraversabilityNodeBase *> trBaseMap(trMap.getNumCells(), trMap.getResolution(), trMap.getLocalMapData());
    
    trBaseMap.maxDist = trMap.maxDist;
    
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

}
