#pragma once

#include <sbpl/discrete_space_information/environment.h>
#include "TraversabilityGenerator3d.hpp"
#include <maps/grid/TraversabilityMap3d.hpp>
#include <base/Pose.hpp>
#include "DiscreteTheta.hpp"
#include "PreComputedMotions.hpp"
#include <base/Trajectory.hpp>


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
    
    RobotModel robotModel;
    PreComputedMotions availableMotions;
    
    ThetaNode *createNewState(const DiscreteTheta& curTheta, EnvironmentXYZTheta::XYZNode* curNode);
    XYZNode *createNewXYZState(TraversabilityGenerator3d::Node* travNode);
    
    ThetaNode *createNewStateFromPose(const Eigen::Vector3d& pos, double theta, EnvironmentXYZTheta::XYZNode** xyzNode);
    
    ThetaNode *startThetaNode;
    XYZNode *startXYZNode;
    ThetaNode *goalThetaNode;
    XYZNode *goalXYZNode;

    int GetHeuristic(int stateID, EnvironmentXYZTheta::ThetaNode* targetThetaNode, EnvironmentXYZTheta::XYZNode* goalXYZNode) const;
 
    void clear();
public:
  
    //FIXME remove all debug code afterwards
    mutable std::vector<maps::grid::Vector3d> debugRobotPositions;
    mutable std::vector<maps::grid::Vector3d> debugColissionCells;
    //contains the min/max vectors for bounding boxes that collided with something
    mutable std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> debugCollisions;
    mutable std::vector<Eigen::Vector4d> debugCost;
    
    Eigen::Vector3d robotHalfSize;
    mutable std::vector<base::Pose> debugCollisionPoses;
    
    mutable std::vector<Eigen::Vector3d> intersectionPositions;

    
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

    const MLGrid &getMlsMap() const;
    
    std::vector<Motion> getMotions(const std::vector<int> &stateIDPath);
    
    void getTrajectory(const std::vector<int> &stateIDPath, std::vector<base::Trajectory> &result);
    
    const PreComputedMotions& getAvailableMotions() const;
private:
  
    //Return true if there is no collision on the given path.
    bool checkCollisions(const std::vector<TraversabilityGenerator3d::Node*>& path,
                         const Motion& motion) const;
  
    Eigen::AlignedBox3d getRobotBoundingBox() const;
    
    void precomputeCost();
    void dijkstraComputeCost(TraversabilityGenerator3d::Node* source, std::vector<double> &cost);
    
    /**Return the avg slope of all patches on the given @p path */
    double getAvgSlope(std::vector<TraversabilityGenerator3d::Node*> path) const;
    
    double getAnglebetweenPlaneAndXY(const Eigen::Hyperplane<double, 3>& plane) const;
    
    TraversabilityGenerator3d::Node* movementPossible(TraversabilityGenerator3d::Node* fromTravNode, const maps::grid::Index& fromIdx, const maps::grid::Index& to);
    TraversabilityConfig travConf;
    
    unsigned int numAngles;
};

}
