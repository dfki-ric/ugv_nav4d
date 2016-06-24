#pragma once

#include <sbpl/discrete_space_information/environment.h>
#include <trav_gen_3d/TraversabilityGenerator3d.hpp>
#include <maps/grid/TraversabilityMap3d.hpp>
#include <base/Pose.hpp>

namespace motion_planning_libraries 
{
    class SbplMotionPrimitives;
}

class EnvironmentXYZTheta : public DiscreteSpaceInformation
{
    TraversabilityGenerator3d travGen;
    boost::shared_ptr<maps::grid::MultiLevelGridMap<maps::grid::SurfacePatchBase> > mlsGrid;

    class PreComputedMotions;
    
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
     
    class DiscreteTheta
    {
        friend class PreComputedMotions;
        int theta;
        unsigned int numAngles;
        
        void normalize()
        {
            if(theta < 0)
                theta += numAngles;

            if(theta >= numAngles)
                theta -= numAngles;
        }
        
    public:
        DiscreteTheta(int val, unsigned int numAngles) : theta(val) , numAngles(numAngles) {
            normalize();
        }
        
        DiscreteTheta(double val, unsigned int numAngles) : numAngles(numAngles) {
            std::cout << "Double constructor called " << val << std::endl;
            theta = floor(val / M_PI / 2.0 * numAngles);
            normalize();
        }
        
        DiscreteTheta(const DiscreteTheta &o) : theta(o.theta), numAngles(o.numAngles) {
        }
        
        DiscreteTheta& operator+=(const DiscreteTheta& rhs)
        {
            theta += rhs.theta;
            normalize();
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
        
        int getTheta() const
        {
          return theta;
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
    
    struct Hash 
    {
        Hash(XYZNode *node, ThetaNode *thetaNode) : node(node), thetaNode(thetaNode)
        {
        }
        XYZNode *node;
        ThetaNode *thetaNode;
    };
    
    std::vector<Hash> idToHash;
    
    struct Motion
    {
        Motion(unsigned int numAngles) : thetaDiff(0, numAngles),startTheta(0, numAngles) {};
        
        int xDiff;
        int yDiff;
        DiscreteTheta thetaDiff;
        DiscreteTheta startTheta;
        
        /**the intermediate poses are not discrete */
        std::vector<base::Pose2D> intermediatePoses;
        
        int baseCost;
    };
     
    
    class PreComputedMotions
    {
        //indexed by discrete start theta
        std::vector<std::vector<Motion> > thetaToMotion;
        
    public:
        void setMotionForTheta(const Motion &motion, const DiscreteTheta &theta);
        
        const std::vector<Motion> &getMotionForStartTheta(DiscreteTheta &theta)
        {
            if(theta.theta >= (int)thetaToMotion.size())
            {
                std::cout << "Input theta is " << theta.theta;
                throw std::runtime_error("Internal error, motion for requested theta ist not available");
            }
            return thetaToMotion.at(theta.theta);
        };
        
        
    };
    
    PreComputedMotions availableMotions;
    
    ThetaNode *createNewState(const EnvironmentXYZTheta::DiscreteTheta& curTheta, EnvironmentXYZTheta::XYZNode* curNode);
    
    ThetaNode *createNewStateFromPose(const Eigen::Vector3d& pos, double theta);
    
    ThetaNode *startNode;
    ThetaNode *goalNode;
    
public:
    EnvironmentXYZTheta(boost::shared_ptr<maps::grid::MultiLevelGridMap<maps::grid::SurfacePatchBase> > mlsGrid,
                        const TraversabilityGenerator3d::Config &travConf,
                        const motion_planning_libraries::SbplMotionPrimitives& primitives);
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
    
    /**Load motion primitives from .mprim file.
     * @ŧhrow EnvironmentXYZThetaException In case of error*/
    virtual void readMotionPrimitives(const std::string& path);
    
    virtual void readMotionPrimitives(const motion_planning_libraries::SbplMotionPrimitives& primitives);
    
    void setStart(const Eigen::Vector3d &startPos, double theta);
    void setGoal(const Eigen::Vector3d &goalPos, double theta);
    
private:
    //note: readVar could be a template but I want to avoid boost dependencies in the header file
    /** Read a variable named @p varName from @p file and store it i @p result */
    void readVar(const std::string& varName, int& result, std::ifstream& file) const;
    void readVar(const std::string& varName, double& result, std::ifstream& file) const;
    void readVar(const std::string& varName, Eigen::Array3i& result, std::ifstream& file) const;
    void readPose2D(base::Pose2D& result, std::ifstream& file) const;  
    
    /**Reads the next primitive from the file.
     * @throw EnvironmentXYZThetaException in case of error*/
    Motion readPrimitive(std::ifstream& file) const;
    
    TraversabilityGenerator3d::Node* movementPossible(TraversabilityGenerator3d::Node* fromTravNode, const maps::grid::Index& fromIdx, const maps::grid::Index& to);
    
    
    const TraversabilityGenerator3d::Config &travConf;
    
    unsigned int numAngles;
};


