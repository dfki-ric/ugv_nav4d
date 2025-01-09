#pragma once

#include <sbpl/discrete_space_information/environment.h>
#undef DEBUG //sbpl defines DEBUG 0 but the word debug is also used in base-logging which is included from TraversabilityGenerator3d
#include <traversability_generator3d/TraversabilityGenerator3d.hpp>
#include <maps/grid/TraversabilityMap3d.hpp>
#include <base/Pose.hpp>
#include "DiscreteTheta.hpp"
#include "PreComputedMotions.hpp"
#include <trajectory_follower/SubTrajectory.hpp>

std::ostream& operator<< (std::ostream& stream, const DiscreteTheta& angle);

namespace ugv_nav4d
{

    class StateCreationFailed : public std::runtime_error {using std::runtime_error::runtime_error;};
    class NodeCreationFailed : public std::runtime_error {using std::runtime_error::runtime_error;};
    class ObstacleCheckFailed : public std::runtime_error {using std::runtime_error::runtime_error;};
    class OrientationNotAllowed : public std::runtime_error {using std::runtime_error::runtime_error;};


class EnvironmentXYZTheta : public DiscreteSpaceInformation
{
protected:

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


    /** A discretized orientation */
    struct ThetaNode
    {
            ThetaNode(const DiscreteTheta &t) :theta(t) {};
            int id;
            DiscreteTheta theta;
    };

    /**PlannerData is the userdata inside the XYZNode. */
    struct PlannerData
    {
        PlannerData() : travNode(nullptr) {};

        /**This is the node that was used to create this XYZNode.*/
        traversability_generator3d::TravGenNode *travNode;

        /**An XYZNode is associated with every ThetaNode that it
         * shares a state with. This map links to all of them,
         * sorted by theta. */
        std::map<DiscreteTheta, ThetaNode *> thetaToNodes;
    };

    /** The distance from somewhere to start-node and goal-node.*/
    struct Distance
    {
        double distToStart = 0;
        double distToGoal = 0;
        Distance(double toStart, double toGoal) : distToStart(toStart), distToGoal(toGoal){}
    };

    /** A position on the traversability map */
    typedef maps::grid::TraversabilityNode<PlannerData> XYZNode;

    //search space without theta
    maps::grid::TraversabilityMap3d<XYZNode *> searchGrid;

    /** Represents one state in the search space */
    struct Hash
    {
        Hash(XYZNode *node, ThetaNode *thetaNode) : node(node), thetaNode(thetaNode)
        {
        }
        XYZNode *node; /**< xyz position of the state and meta data */
        ThetaNode *thetaNode;/** < angle of the state and additional meta data */
    };

    /**maps sbpl state ids to internal planner state (Hash). */
    std::vector<Hash> idToHash;

    /**Contains the distance from each travNode to start-node and goal-node
     * Stored in real-world coordinates (i.e. do NOT scale with gridResolution before use)*/
    std::vector<Distance> travNodeIdToDistance;
    std::shared_ptr<const traversability_generator3d::TravMap3d > travMap;
    PreComputedMotions availableMotions;

    ThetaNode *startThetaNode;
    XYZNode *startXYZNode; //part of the start state
    ThetaNode *goalThetaNode;
    XYZNode *goalXYZNode; //part of the goal state

    ThetaNode *createNewState(const DiscreteTheta& curTheta, EnvironmentXYZTheta::XYZNode* curNode);
    XYZNode *createNewXYZState(traversability_generator3d::TravGenNode* travNode);
    ThetaNode *createNewStateFromPose(const std::string& name, const Eigen::Vector3d& pos, double theta, ugv_nav4d::EnvironmentXYZTheta::XYZNode** xyzBackNode);

    bool checkStartGoalNode(const std::string& name, traversability_generator3d::TravGenNode* node, double theta);

public:

    /** @param pos Position in map frame */
    bool obstacleCheck(const maps::grid::Vector3d& pos, double theta, 
                              const traversability_generator3d::TraversabilityConfig& travConf,
                              const sbpl_spline_primitives::SplinePrimitivesConfig& splineConf,
                              const std::string& nodeName="node");
    Eigen::Vector3d robotHalfSize;

    /** @param generateDebugData If true, lots of debug information will becollected
     *                           and stored in members starting with debug*/
    EnvironmentXYZTheta(std::shared_ptr<const traversability_generator3d::TravMap3d > travMap,
                        const traversability_generator3d::TraversabilityConfig &travConf,
                        const sbpl_spline_primitives::SplinePrimitivesConfig &primitiveConfig,
                        const Mobility& mobilityConfig);

    virtual ~EnvironmentXYZTheta();

    void updateMap(std::shared_ptr<const traversability_generator3d::TravMap3d > travMap);

    virtual bool InitializeEnv(const char* sEnvFile);
    virtual bool InitializeMDPCfg(MDPConfig* MDPCfg);

    /**Returns the trajectory of least resistance to leave the obstacle.
     * @param start start position that is inside an obstacle
     * @param theta robot orientation
     * @param[out] outNewStart The new start position of the robot after it has moved out of the obstacle in the map frame
     * @return the best trajectory that gets the robot out of the obstacle.
     *         Or an empty trajectory if no way out can be found*/
    std::shared_ptr<trajectory_follower::SubTrajectory> findTrajectoryOutOfObstacle(const Eigen::Vector3d& start, double theta,
            const Eigen::Affine3d& ground2Body, bool setZToZero);


     /**
     * \brief heuristic estimate from state FromStateID to state ToStateID
     */
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID);
    /**
     * \brief heuristic estimate from start state to state with stateID
     */
    virtual int GetStartHeuristic(int stateID);
    /**
     *
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

    traversability_generator3d::TravGenNode* findMatchingTraversabilityPatchAt(const Eigen::Vector3d &pos);

    /**returns the motion connection @p fromStateID and @p toStateID.
     * @throw std::runtime_error if no matching motion exists*/
    const Motion& getMotion(const int fromStateID, const int toStateID);

    const std::shared_ptr<const traversability_generator3d::TravMap3d> getTraversabilityMap() const;

    std::vector<Motion> getMotions(const std::vector<int> &stateIDPath);

    void getTrajectory(const std::vector<int> &stateIDPath, std::vector<trajectory_follower::SubTrajectory> &result,
                       bool setZToZero, const Eigen::Vector3d &startPos, const Eigen::Vector3d &goalPos, const double& goalHeading, const Eigen::Affine3d &plan2Body = Eigen::Affine3d::Identity());

    const PreComputedMotions& getAvailableMotions() const;

    /**Clears the state of the environment. */
    void clear();

    void setTravConfig(const traversability_generator3d::TraversabilityConfig& cfg);

    /** @param maxDist The value that should be used as maximum distance. This value is used for
     *                 non-traversable nodes and for initialization.*/
    void dijkstraComputeCost(const traversability_generator3d::TravGenNode* source, std::vector<double> &outDistances,
                             const double maxDist) const;

    /** Should a computationally expensive obstacle check be done to check whether the robot bounding box
     *  is in collision with obstacles. This mode is useful for highly cluttered and tight spaced environments */
    void enablePathStatistics(bool enable);

private:

    /** Check if all nodes on the path from @p sourceNode following @p motion are traversable.
     * @return the target node of the motion or nullptr if motion not possible */
    traversability_generator3d::TravGenNode* checkTraversableHeuristic(const maps::grid::Index sourceIndex, traversability_generator3d::TravGenNode* sourceNode,
                                           const ugv_nav4d::Motion& motion, const maps::grid::TraversabilityMap3d< traversability_generator3d::TravGenNode* >& trMap);

    /** Some movement directions are not allowed depending on the slope of the patch.
     *  @return true if the movement direction is allowed on that patch
     */
    bool checkOrientationAllowed(const traversability_generator3d::TravGenNode* node,
                                 const base::Orientation2D& orientation) const;


    /** Computes the heuristic */
    void precomputeCost();

    /**Return the avg slope of all patches on the given @p path */
    double getAvgSlope(std::vector<const traversability_generator3d::TravGenNode*> path) const;

    /**Returns the max slope of all patches on the given @p path */
    double getMaxSlope(std::vector<const traversability_generator3d::TravGenNode*> path) const;


    /**Determines the distance between @p a and @p b depending on travConf.heuristicType */
    double getHeuristicDistance(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const;

    /** Checks if movement from @p fromTravNode to its neighbor at position @p toIdx is possible.
     *  I.e. if a direct connection exists and if the neighbor is traversable.
     *  Expands the neighbor if not already expanded.
     *  @return the neighbor at position @p toIdx */
    traversability_generator3d::TravGenNode* movementPossible(traversability_generator3d::TravGenNode* fromTravNode, const maps::grid::Index& fromIdx, const maps::grid::Index& toIdx);

    /** Expands @p node if it needs expansion.
     *  Thread-safe.
     *  @return True if the expansion succeeded */
    bool checkExpandTreadSafe(traversability_generator3d::TravGenNode * node);

    bool usePathStatistics;

    traversability_generator3d::TraversabilityConfig travConf;
    sbpl_spline_primitives::SplinePrimitivesConfig primitiveConfig;

    unsigned int numAngles;

    Mobility mobilityConfig;
};

}
