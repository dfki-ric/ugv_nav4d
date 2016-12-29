#pragma once

#include <sbpl/discrete_space_information/environment.h>
#include "TraversabilityGenerator3d.hpp"
#include <maps/grid/TraversabilityMap3d.hpp>
#include <base/Pose.hpp>
#include "DiscreteTheta.hpp"
#include "PreComputedMotions.hpp"
#include <base/Trajectory.hpp>
#include <unordered_set>


std::ostream& operator<< (std::ostream& stream, const DiscreteTheta& angle);

namespace ugv_nav4d
{

class EnvironmentXYZTheta : public DiscreteSpaceInformation
{
public:
    typedef maps::grid::MultiLevelGridMap< maps::grid::SurfacePatchBase > MLGrid;
protected:
    TraversabilityGenerator3d travGen;
    boost::shared_ptr<MLGrid > mlsGrid;

    struct EnvironmentXYZThetaException : public SBPL_Exception
    {
      EnvironmentXYZThetaException(const std::string& what) : 
          msg("SBPL has encountered a fatal error: " + what){}
      
        virtual const char* what() const throw()
        {
            return msg.c_str();
        }
      const std::string msg;
    };
     

    struct ThetaNode
    {
            ThetaNode(const DiscreteTheta &t) :theta(t) {};
            int id;
            DiscreteTheta theta;
    };
    
    struct PlannerData
    {
        PlannerData() : travNode(nullptr) {};
        
        TraversabilityGenerator3d::Node *travNode;
        
        ///contains all nodes sorted by theta
        std::map<DiscreteTheta, ThetaNode *> thetaToNodes; 
    };
    
    struct Distance
    {
        double distToStart = 0;
        double distToGoal = 0;
        Distance(double toStart, double toGoal) : distToStart(toStart), distToGoal(toGoal){}
    };
    
    typedef maps::grid::TraversabilityNode<PlannerData> XYZNode;
    
    maps::grid::TraversabilityMap3d<XYZNode *> searchGrid;
    
    struct Hash 
    {
        Hash(XYZNode *node, ThetaNode *thetaNode) : node(node), thetaNode(thetaNode)
        {
        }
        XYZNode *node;
        ThetaNode *thetaNode;
    };
    
    std::vector<Hash> idToHash;
    /**Contains the distance from each travNode to start and goal
     * Stored in real-world coordinates (i.e. do NOT scale with gridResolution before use)*/
    std::vector<Distance> travNodeIdToDistance;
    
    PreComputedMotions availableMotions;
    
    ThetaNode *startThetaNode;
    XYZNode *startXYZNode;
    ThetaNode *goalThetaNode;
    XYZNode *goalXYZNode;
    
    ThetaNode *createNewState(const DiscreteTheta& curTheta, EnvironmentXYZTheta::XYZNode* curNode);
    XYZNode *createNewXYZState(TraversabilityGenerator3d::Node* travNode);
    ThetaNode *createNewStateFromPose(const Eigen::Vector3d& pos, double theta, EnvironmentXYZTheta::XYZNode** xyzNode);
    
public:
    mutable std::vector<Eigen::Vector4d> debugHeuristic; /**< The heuristic [x, y, z, cost]. (in world coordinates) */
    mutable std::vector<base::Pose> debugCollisionPoses; /**< Poses of collisions that occured while planning (in world coordinates) */
    mutable std::vector<Eigen::Vector3d> debugSuccessors; /**< All positions that the planner visited while planning in chronological order (in world coordinates) */
    
    struct DebugSlopeData
    {
        Eigen::Vector3d start, end1, end2, end3, end4;
        maps::grid::Index i;
        bool operator==(const DebugSlopeData& other) const
        {
            return i.x() == other.i.x() && i.y() == other.i.y();
        }
    };
    
    struct DebugSlopeData_hash
    {
        std::size_t operator()(const DebugSlopeData& p) const
        {
            return std::hash<int>()(p.i.x()) ^ std::hash<int>()(p.i.y());
        }
    };

    mutable std::unordered_set<DebugSlopeData, DebugSlopeData_hash> debugSlopeData;
    
    struct DebugSlopeCandidate
    {
        Eigen::Vector3d start, end;
        maps::grid::Index i;
        double orientation;
        enum SLOP_COL { RED, GREEN};
        SLOP_COL color;
        bool operator==(const DebugSlopeCandidate& other) const
        {
            return i.x() == other.i.x() && i.y() == other.i.y() &&
                   int(orientation * 100) == int(other.orientation * 100);
        }
    };
    
    struct DebugSlopeCandidate_hash
    {
        std::size_t operator()(const DebugSlopeCandidate& p) const
        {
//              std::cout << "Hash: " << p.i.transpose() << ", " << p.orientation << ", " << (int)(p.orientation * 100) << std::endl;
            return std::hash<int>()(p.i.x()) ^ std::hash<int>()(p.i.y()) ^ std::hash<int>()((int)(p.orientation * 100));
        }
    };
    
    mutable std::unordered_set<DebugSlopeCandidate, DebugSlopeCandidate_hash> debugSlopeCandidates;

    Eigen::Vector3d robotHalfSize;
    
    /** @param generateDebugData If true, lots of debug information will becollected
     *                           and stored in members starting with debug*/
    EnvironmentXYZTheta(boost::shared_ptr<MLGrid > mlsGrid,
                        const TraversabilityConfig &travConf,
                        const motion_planning_libraries::SplinePrimitivesConfig &primitiveConfig,
                        const motion_planning_libraries::Mobility& mobilityConfig);
    
    virtual ~EnvironmentXYZTheta();
    
    void updateMap(boost::shared_ptr<MLGrid > mlsGrid);
    
    virtual bool InitializeEnv(const char* sEnvFile);
    virtual bool InitializeMDPCfg(MDPConfig* MDPCfg);
    
     /**
     * \brief heuristic estimate from state FromStateID to state ToStateID
     */
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID);
    /**
     * \brief heuristic estimate from start state to state with stateID
     */
    virtual int GetStartHeuristic(int stateID);
    /**
     * \brief heuristic estimate from state with stateID to goal state
     */
    virtual int GetGoalHeuristic(int stateID);
    
    virtual void GetPreds(int TargetStateID, std::vector< int >* PredIDV, std::vector< int >* CostV);
    virtual void GetSuccs(int SourceStateID, std::vector< int >* SuccIDV, std::vector< int >* CostV);
    virtual void GetSuccs(int SourceStateID, std::vector< int >* SuccIDV, std::vector< int >* CostV, std::vector< size_t >& motionIdV);

    virtual void PrintEnv_Config(FILE* fOut);
    virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = 0);

    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);
    virtual void SetAllPreds(CMDPSTATE* state);
    virtual int SizeofCreatedEnv();
    
    void setStart(const Eigen::Vector3d &startPos, double theta);
    void setGoal(const Eigen::Vector3d &goalPos, double theta);
    
    maps::grid::Vector3d getStatePosition(const int stateID) const;
    
    /**Returns the intermediate poses of the motion connecting @p FromStateID 
     * and @p toStateID.
     * @throw std::runtime_error if no such motion exists*/
    const std::vector<PoseWithCell> &getPoses(const int fromStateID, const int toStateID);
    
    /**returns the motion connection @p fromStateID and @p toStateID.
     * @throw std::runtime_error if no matching motion exists*/
    const Motion& getMotion(const int fromStateID, const int toStateID);
    
    maps::grid::TraversabilityMap3d< maps::grid::TraversabilityNodeBase* > getTraversabilityBaseMap() const;
    const maps::grid::TraversabilityMap3d< TraversabilityGenerator3d::Node *> &getTraversabilityMap() const;
    TraversabilityGenerator3d& getTravGen();

    const MLGrid &getMlsMap() const;
    
    std::vector<Motion> getMotions(const std::vector<int> &stateIDPath);
    
    void getTrajectory(const std::vector<int> &stateIDPath, std::vector<base::Trajectory> &result);
    
    const PreComputedMotions& getAvailableMotions() const;
    
    /**Clears the state of the environment. Clears everything except the mls map. */
    void clear();
    
    void setTravConfig(const TraversabilityConfig& cfg);
    
private:
  
    //Return true if there is no collision on the given path.
    bool checkCollisions(const std::vector<TraversabilityGenerator3d::Node*>& path,
                         const Motion& motion) const;
                         
    /** Some movement directions are not allowed depending on the slope of the patch.
     *  @return true if the movement direction is allowed on that patch
     */
    bool checkOrientationAllowed(const TraversabilityGenerator3d::Node* node,
                                 const base::Orientation2D& orientation) const;
  
    Eigen::AlignedBox3d getRobotBoundingBox() const;
    
    void precomputeCost();
    /** @param maxDist The value that should be used as maximum distance. This value is used for
     *                 non-traversable nodes and for initialization.*/ 
    void dijkstraComputeCost(TraversabilityGenerator3d::Node* source, std::vector<double> &outDistances,
                             const double maxDist);
    
    /**Return the avg slope of all patches on the given @p path */
    double getAvgSlope(std::vector<TraversabilityGenerator3d::Node*> path) const;
    
    TraversabilityGenerator3d::Node* movementPossible(TraversabilityGenerator3d::Node* fromTravNode, const maps::grid::Index& fromIdx, const maps::grid::Index& to);
    TraversabilityConfig travConf;
    
    unsigned int numAngles;
    
    motion_planning_libraries::Mobility mobilityConfig;
};

}
