#pragma once
#include "TraversabilityConfig.hpp"
#include "TraversabilityGenerator3d.hpp"
#include <unordered_set>
#include <unordered_map>
 
namespace ugv_nav4d_debug
{

struct DebugSlopeData
{
    Eigen::Vector3d start, end1, end2, end3, end4;
    maps::grid::Index i;
    bool operator==(const DebugSlopeData& other) const
    {
        return i.x() == other.i.x() && i.y() == other.i.y();
    }
};

struct DebugSlopeDataHash
{
    std::size_t operator()(const DebugSlopeData& p) const
    {
        return std::hash<int>()(p.i.x()) ^ std::hash<int>()(p.i.y());
    }
};

struct DebugSlopeCandidate
{
    Eigen::Vector3d start, end;
    maps::grid::Index i;
    double orientation;
    enum SlopeColor { RED, GREEN};
    SlopeColor color;
    bool operator==(const DebugSlopeCandidate& other) const
    {
        return i.x() == other.i.x() && i.y() == other.i.y() &&
                int(orientation * 100) == int(other.orientation * 100);
    }
};

struct DebugSlopeCandidateHash
{
    std::size_t operator()(const DebugSlopeCandidate& p) const
    {
        //this ensures that there very similar orientations on the same patch are grouped.
        //This is done to avoid graphics lag caused by drawing too many lines
        return std::hash<int>()(p.i.x()) ^ std::hash<int>()(p.i.y()) ^ std::hash<int>()((int)(p.orientation * 100));
    }
};
    
class EnvironmentXYZThetaDebugData
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    void setTravConf(const ugv_nav4d::TraversabilityConfig& conf) 
    {
        travConf = conf;
    }
    
    void setMlsGrid(boost::shared_ptr<maps::grid::MultiLevelGridMap< maps::grid::SurfacePatchBase>> mls)
    {
        mlsGrid = mls;
    }
    
    void setTravGen(ugv_nav4d::TraversabilityGenerator3d* gen)
    {
        travGen = gen;
    }
    
    void addSucc(ugv_nav4d::TravGenNode* node)
    {
        Eigen::Vector3d succ((node->getIndex().x() + 0.5) * travConf.gridResolution,
                             (node->getIndex().y() + 0.5) * travConf.gridResolution,
                              node->getHeight());
    
    successors.push_back(mlsGrid->getLocalFrame().inverse(Eigen::Isometry) * succ);
    }
    
    void orientationCheck(const ugv_nav4d::TravGenNode* node, const base::AngleSegment& seg1, const base::AngleSegment& seg2, const base::Angle& orientation, bool candidateInside)
    {
        DebugSlopeData d;
        d.start << (node->getIndex().x() + 0.5) * travConf.gridResolution, (node->getIndex().y() + 0.5) * travConf.gridResolution, node->getHeight();
        d.i = node->getIndex();
        
        const Eigen::Vector3d one(0.05, 0, 0);
        d.end1 = d.start + Eigen::AngleAxisd(seg1.getStart().rad, Eigen::Vector3d(0,0,1)) * one;
        d.end2 = d.start + Eigen::AngleAxisd(seg1.getEnd().rad, Eigen::Vector3d(0,0,1)) * one;
        d.end3 = d.start + Eigen::AngleAxisd(seg2.getStart().rad, Eigen::Vector3d(0,0,1)) * one;
        d.end4 = d.start + Eigen::AngleAxisd(seg2.getEnd().rad, Eigen::Vector3d(0,0,1)) * one;
        
        DebugSlopeCandidate candidate;
        candidate.start = d.start;
        candidate.end = candidate.start + Eigen::AngleAxisd(orientation.rad, Eigen::Vector3d(0,0,1)) * one;
        candidate.i = node->getIndex();
        candidate.orientation = orientation.rad;
        
        //convert to local map frame
        candidate.end = mlsGrid->getLocalFrame().inverse(Eigen::Isometry) * candidate.end;
        candidate.start = mlsGrid->getLocalFrame().inverse(Eigen::Isometry) * candidate.start;
        d.start = mlsGrid->getLocalFrame().inverse(Eigen::Isometry) * d.start;
        d.end1 = mlsGrid->getLocalFrame().inverse(Eigen::Isometry) * d.end1;
        d.end2 = mlsGrid->getLocalFrame().inverse(Eigen::Isometry) * d.end2;
        d.end3 = mlsGrid->getLocalFrame().inverse(Eigen::Isometry) * d.end3;
        d.end4 = mlsGrid->getLocalFrame().inverse(Eigen::Isometry) * d.end4;
        
        if(candidateInside)
            candidate.color = DebugSlopeCandidate::GREEN;
        else
            candidate.color = DebugSlopeCandidate::RED;
            
        slopeData.insert(d);
        slopeCandidates.insert(candidate);
    }
    
    void addCollision(const maps::grid::Vector3d& robotPosition, const Eigen::Quaterniond& robotRotation)
    {
        //FIXME I guess robotRotation has to be rotate by the mlsGrid local Frame aswell?
        collisions.push_back(base::Pose(mlsGrid->getLocalFrame().inverse(Eigen::Isometry) * robotPosition, robotRotation));
    }
    
    void clearHeuristic()
    {
        heuristicCost.clear();
    }
    
    void addHeuristicCost(ugv_nav4d::TravGenNode* node, double cost)
    {
        if(heuristicCost.find(node) == heuristicCost.end())
        {
            heuristicCost[node] = cost;
        }
        else
        {
            if(heuristicCost[node] > cost)
            {
                heuristicCost[node] = cost;
            }
        }
    }
    
    //[0..2] = position, [3] = cost
    std::vector<Eigen::Vector4d> getHeuristicCostForViz()
    {
        std::vector<Eigen::Vector4d> output;
        for(auto i : heuristicCost)
        {
            const ugv_nav4d::TravGenNode* node = i.first;
            const double cost = i.second;
            Eigen::Vector3d pos(node->getIndex().x() * travConf.gridResolution, node->getIndex().y() * travConf.gridResolution, node->getHeight());
            pos = travGen->getTraversabilityMap().getLocalFrame().inverse(Eigen::Isometry) * pos;
            output.push_back(Eigen::Vector4d(pos.x(), pos.y(), pos.z(), cost));
        }
        return output;
    }
    
    std::vector<Eigen::Vector3d>& getSuccs()
    {
        return successors;
    }
    
    std::vector<base::Pose>& getCollisions()
    {
        return collisions;
    }
    
    const std::unordered_set<DebugSlopeData, DebugSlopeDataHash>& getSlopeData()
    {
        return slopeData;
    }
    
    const std::unordered_set<DebugSlopeCandidate, DebugSlopeCandidateHash>& getSlopeCandidates()
    {
        return slopeCandidates;
    }
    
private:
    ugv_nav4d::TraversabilityConfig travConf;
    
    /**All successors that are visited during exploration */
    std::vector<Eigen::Vector3d> successors;
    boost::shared_ptr<maps::grid::MultiLevelGridMap< maps::grid::SurfacePatchBase>> mlsGrid;
    ugv_nav4d::TraversabilityGenerator3d* travGen;
    
    std::unordered_set<DebugSlopeData, DebugSlopeDataHash> slopeData;
    std::unordered_set<DebugSlopeCandidate, DebugSlopeCandidateHash> slopeCandidates;
    std::vector<base::Pose> collisions;
    std::unordered_map<ugv_nav4d::TravGenNode*, double> heuristicCost;
};

}
    
 