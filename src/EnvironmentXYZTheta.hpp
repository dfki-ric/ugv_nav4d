#pragma once

#include <sbpl/discrete_space_information/environment.h>
#include <trav_gen_3d/TraversabilityGenerator3d.hpp>
#include <maps/grid/TraversabilityMap3d.hpp>
#include <base/Pose.hpp>

namespace motion_planning_libraries 
{
    class SbplMotionPrimitives;
}

class DiscreteTheta
{
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

    DiscreteTheta& operator-=(const DiscreteTheta& rhs)
    {
        theta -= rhs.theta;
        normalize();
        return *this;
    }
    
    friend DiscreteTheta operator+(DiscreteTheta lhs, const DiscreteTheta& rhs)
    {
        lhs += rhs;
        return lhs;
    }

    friend DiscreteTheta operator-(DiscreteTheta lhs, const DiscreteTheta& rhs)
    {
        lhs -= rhs;
        return lhs;
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
    
    double getRadian() const
    {
        return M_PI * 2.0 * theta / static_cast<double>(numAngles);
    }
    
    
    DiscreteTheta shortestDist(const DiscreteTheta &ain) const
    {
        DiscreteTheta diffA = ain-*this;
        
        int a = diffA.theta;
        int b = numAngles - diffA.theta;
        
        
        if(a < b)
            return DiscreteTheta(a, numAngles);
        
        return DiscreteTheta(b, numAngles);
    }
};

std::ostream& operator<< (std::ostream& stream, const DiscreteTheta& angle)
{
    stream << angle.getTheta();
    return stream;
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
        Motion(unsigned int numAngles) : endTheta(0, numAngles),startTheta(0, numAngles), baseCost(0) {};
        
        int xDiff;
        int yDiff;
        DiscreteTheta endTheta;
        DiscreteTheta startTheta;
        
        /**the intermediate poses are not discrete.
         * They are relative to the starting cell*/
        std::vector<base::Pose2D> intermediatePoses;
        /**relative to starting cell */
        std::vector<maps::grid::Index> intermediateCells;
        
        int baseCost;
        
        int costMultiplier;
    };
     
    
    class RobotModel
    {
    public:
        RobotModel(double tr, double rv);
        
        ///in m per sec
        double translationalVelocity;
        ///in rad per sec
        double rotationalVelocity;
    };
    
    class PreComputedMotions
    {
        //indexed by discrete start theta
        std::vector<std::vector<Motion> > thetaToMotion;
        
    public:
        void setMotionForTheta(const Motion &motion, const DiscreteTheta &theta);
        
        void preComputeCost(Motion &motion, const RobotModel &model);
        
        const std::vector<Motion> &getMotionForStartTheta(DiscreteTheta &theta)
        {
            if(theta.getTheta() >= (int)thetaToMotion.size())
            {
                std::cout << "Input theta is " << theta.getTheta();
                throw std::runtime_error("Internal error, motion for requested theta ist not available");
            }
            return thetaToMotion.at(theta.getTheta());
        };
        
        
    };
    
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
    
    
    virtual void readMotionPrimitives(const motion_planning_libraries::SbplMotionPrimitives& primitives);
    
    void setStart(const Eigen::Vector3d &startPos, double theta);
    void setGoal(const Eigen::Vector3d &goalPos, double theta);
    
private:
    
    TraversabilityGenerator3d::Node* movementPossible(TraversabilityGenerator3d::Node* fromTravNode, const maps::grid::Index& fromIdx, const maps::grid::Index& to);
    const TraversabilityGenerator3d::Config &travConf;
    
    unsigned int numAngles;
};


