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

protected:
    TraversabilityGenerator3d travGen;
    boost::shared_ptr<maps::grid::MultiLevelGridMap<maps::grid::SurfacePatchBase> > mlsGrid;

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
     

    class ThetaNode
    {
        public:
            ThetaNode(const DiscreteTheta &t) :theta(t) {};
            int id;
            DiscreteTheta theta;
    };
    
    class PlannerData
    {
    public:
        PlannerData() : travNode(nullptr) {};
        
        TraversabilityGenerator3d::Node *travNode;
        
        ///contains all nodes sorted by theta
        std::map<DiscreteTheta, ThetaNode *> thetaToNodes; 
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
    
    Eigen::Vector3d robotHalfSize;
    mutable std::vector<base::Pose> debugCollisionPoses;
    
    mutable std::vector<Eigen::Vector3d> intersectionPositions;

    
    EnvironmentXYZTheta(boost::shared_ptr<maps::grid::MultiLevelGridMap<maps::grid::SurfacePatchBase> > mlsGrid,
                        const TraversabilityConfig &travConf,
                        const motion_planning_libraries::SplinePrimitivesConfig &primitiveConfig,
                        const motion_planning_libraries::Mobility& mobilityConfig);
    
    virtual ~EnvironmentXYZTheta();
    
    void updateMap(boost::shared_ptr<maps::grid::MultiLevelGridMap<maps::grid::SurfacePatchBase> > mlsGrid);
    
    virtual bool InitializeEnv(const char* sEnvFile);
    virtual bool InitializeMDPCfg(MDPConfig* MDPCfg);
    
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID);
    virtual int GetStartHeuristic(int stateID);
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

    const maps::grid::MultiLevelGridMap<maps::grid::SurfacePatchBase> &getMlsMap() const;
    
    std::vector<Motion> getMotions(const std::vector<int> &stateIDPath);
    
    void getTrajectory(const std::vector<int> &stateIDPath, std::vector<base::Trajectory> &result);
    
    const PreComputedMotions& getAvailableMotions() const;
private:
  
    //Return true if there is no collision on the given path.
    bool checkCollisions(const std::vector<TraversabilityGenerator3d::Node*>& path,
                         const Motion& motion) const;
  
    Eigen::AlignedBox3d getRobotBoundingBox() const;
    
    TraversabilityGenerator3d::Node* movementPossible(TraversabilityGenerator3d::Node* fromTravNode, const maps::grid::Index& fromIdx, const maps::grid::Index& to);
    TraversabilityConfig travConf;
    
    unsigned int numAngles;
};

}
