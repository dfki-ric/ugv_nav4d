#pragma once

#include <sbpl/discrete_space_information/environment.h>
#include <trav_gen_3d/TraversabilityGenerator3d.hpp>
#include <maps/grid/TraversabilityMap3d.hpp>

class EnvironmentXYZTheta : public DiscreteSpaceInformation
{
    TraversabilityGenerator3d travGen;
    boost::shared_ptr<maps::grid::MultiLevelGridMap<maps::grid::SurfacePatchBase> > mlsGrid;

    class PreComputedMotions;
     
    class DiscreteTheta
    {
        friend class PreComputedMotions;
        int theta;
    public:
        DiscreteTheta(int val) : theta(val) {};
        
        DiscreteTheta& operator+=(const DiscreteTheta& rhs)
        {
            theta += rhs.theta;
            return *this;
        }
        
        friend DiscreteTheta operator+(DiscreteTheta lhs,        // passing lhs by value helps optimize chained a+b+c
                     const DiscreteTheta& rhs) // otherwise, both parameters may be const references
        {
            lhs += rhs; // reuse compound assignment
            return lhs; // return the result by value (uses move constructor)
        }
        
        friend bool operator<(const DiscreteTheta& l, const DiscreteTheta& r)
        {
            return l.theta < r.theta;
        }

        friend bool operator==(const DiscreteTheta& l, const DiscreteTheta& r)
        {
            return l.theta == r.theta;
        }
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
        
        ///contains alle nodes sorted by theta
        std::map<DiscreteTheta, ThetaNode *> thetaToNodes; 
    };
    
    typedef maps::grid::TraversabilityNode<PlannerData> XYZNode;
    
    maps::grid::TraversabilityMap3d<XYZNode *> searchGrid;
    
    class Hash 
    {
    public:
        Hash(XYZNode *node, ThetaNode *thetaNode) : node(node), thetaNode(thetaNode)
        {
        }
        XYZNode *node;
        ThetaNode *thetaNode;
    };
    
    std::vector<Hash> idToHash;
    
    class Motion
    {
    public:
        Motion() : thetaDiff(0) {};
        
        int xDiff;
        int yDiff;
        DiscreteTheta thetaDiff;
        
        std::vector<Eigen::Vector2i> intermediateCells;
        
        int baseCost;
    };
     
    
    class PreComputedMotions
    {
        std::vector<std::vector<Motion> > thetaToMotion;
        
    public:
        void setMotionForTheta(const Motion &motion, const DiscreteTheta &theta);
        
        const std::vector<Motion> &getMotionForStartTheta(DiscreteTheta &theta)
        {
            if(theta.theta > thetaToMotion.size())
            {
                throw std::runtime_error("Internal error, motion for requested theta ist not available");
            }
            return thetaToMotion[theta.theta];
        };
        
        
    };
    
    PreComputedMotions availableMotions;
    
    ThetaNode *createNewState(const EnvironmentXYZTheta::DiscreteTheta& curTheta, EnvironmentXYZTheta::XYZNode* curNode);
    
    ThetaNode *createNewStateFromPose(const Eigen::Vector3d& pos, double theta);
    
    ThetaNode *startNode;
    ThetaNode *goalNode;
    
public:
    EnvironmentXYZTheta(boost::shared_ptr<maps::grid::MultiLevelGridMap<maps::grid::SurfacePatchBase> > mlsGrid, const TraversabilityGenerator3d::Config &travConf);
    virtual ~EnvironmentXYZTheta();
    
    virtual bool InitializeEnv(const char* sEnvFile);
    virtual bool InitializeMDPCfg(MDPConfig* MDPCfg);
    
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID);
    virtual int GetStartHeuristic(int stateID);
    virtual int GetGoalHeuristic(int stateID);
    
    virtual void GetPreds(int TargetStateID, std::vector< int >* PredIDV, std::vector< int >* CostV);
    virtual void GetSuccs(int SourceStateID, std::vector< int >* SuccIDV, std::vector< int >* CostV);

    virtual void PrintEnv_Config(FILE* fOut);
    virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = 0);

    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);
    virtual void SetAllPreds(CMDPSTATE* state);
    virtual int SizeofCreatedEnv();
    
    void setStart(const Eigen::Vector3d &startPos, double theta);
    void setGoal(const Eigen::Vector3d &goalPos, double theta);
};


