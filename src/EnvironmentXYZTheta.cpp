#include "EnvironmentXYZTheta.hpp"
#include <sbpl/planners/planner.h>
#include <sbpl/utils/mdpconfig.h>

void EnvironmentXYZTheta::PreComputedMotions::setMotionForTheta(const EnvironmentXYZTheta::Motion& motion, const EnvironmentXYZTheta::DiscreteTheta& theta)
{
    if(thetaToMotion.size() <= theta.theta)
    {
        thetaToMotion.resize(theta.theta + 1);
    }
    
    thetaToMotion[theta.theta].push_back(motion);
}


EnvironmentXYZTheta::EnvironmentXYZTheta(boost::shared_ptr< maps::grid::MultiLevelGridMap< maps::grid::SurfacePatchBase > > mlsGrid, const TraversabilityGenerator3d::Config &travConf) : 
    travGen(travConf)
    , mlsGrid(mlsGrid)
    , startNode(nullptr)
    , goalNode(nullptr)
{
    travGen.setMLSGrid(mlsGrid);
    
    searchGrid.setResolution(Eigen::Vector2d(travConf.gridResolution, travConf.gridResolution));
    searchGrid.extend(travGen.getTraversabilityMap().getNumCells());
    
    
    Motion simpleForward;
    
    simpleForward.baseCost = 1;
    simpleForward.xDiff = 1;
    simpleForward.yDiff = 0;
    simpleForward.thetaDiff = 0;

    simpleForward.intermediateCells.push_back(Eigen::Vector2i(1,0));
    
    availableMotions.setMotionForTheta(simpleForward, DiscreteTheta(0));
}

EnvironmentXYZTheta::~EnvironmentXYZTheta()
{

}

EnvironmentXYZTheta::ThetaNode* EnvironmentXYZTheta::createNewStateFromPose(const Eigen::Vector3d& pos, double theta)
{
    TraversabilityGenerator3d::Node *travNode = travGen.generateStartNode(pos);
    if(!travNode)
    {
        std::cout << "createNewStateFromPose: Error Pose " << pos.transpose() << " is out of grid" << std::endl;
        throw std::runtime_error("Pose is out of grid");
    }
    
    XYZNode *xyzNode = new XYZNode(travNode->getHeight(), travNode->getIndex());
    xyzNode->getUserData().travNode = travNode;
    searchGrid.at(travNode->getIndex()).insert(xyzNode);
    
    DiscreteTheta thetaD(0);
    
    return createNewState(thetaD, xyzNode);
}


void EnvironmentXYZTheta::setGoal(const Eigen::Vector3d& goalPos, double theta)
{
    goalNode = createNewStateFromPose(goalPos, theta);
}

void EnvironmentXYZTheta::setStart(const Eigen::Vector3d& startPos, double theta)
{
    startNode = createNewStateFromPose(startPos, theta);
}

void EnvironmentXYZTheta::SetAllPreds(CMDPSTATE* state)
{
    //implement this if the planner needs access to predecessors

    SBPL_ERROR("ERROR in EnvNAV2D... function: SetAllPreds is undefined\n");
    throw new SBPL_Exception();
}

void EnvironmentXYZTheta::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
    SBPL_ERROR("ERROR in EnvNAV2D... function: SetAllActionsandAllOutcomes is undefined\n");
    throw new SBPL_Exception();
}

int EnvironmentXYZTheta::GetFromToHeuristic(int FromStateID, int ToStateID)
{
    //dummy implementation
    return 0;
}

int EnvironmentXYZTheta::GetGoalHeuristic(int stateID)
{
    //dummy implementation
    return 0;
}

int EnvironmentXYZTheta::GetStartHeuristic(int stateID)
{
    //dummy implementation
    return 0;
}

bool EnvironmentXYZTheta::InitializeEnv(const char* sEnvFile)
{
    return true;
}

bool EnvironmentXYZTheta::InitializeMDPCfg(MDPConfig* MDPCfg)
{
    if(!goalNode || !startNode)
        return false;
    
    //initialize MDPCfg with the start and goal ids
    MDPCfg->goalstateid = goalNode->id;
    MDPCfg->startstateid = startNode->id;

    return true;
}

EnvironmentXYZTheta::ThetaNode *EnvironmentXYZTheta::createNewState(const DiscreteTheta &curTheta, XYZNode *curNode)
{
    ThetaNode *newNode = new ThetaNode(curTheta);
    newNode->id = idToHash.size();
    Hash hash(curNode, newNode);
    idToHash.push_back(hash);
    curNode->getUserData().thetaToNodes.insert(std::make_pair(curTheta, newNode));
    
    //this structure need to be extended for every new state that is added. 
    //Is seems it is later on filled in by the planner.
    
    //insert into and initialize the mappings
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for (int i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
        StateID2IndexMapping[newNode->id][i] = -1;
    }
    
    return newNode;
}


void EnvironmentXYZTheta::GetSuccs(int SourceStateID, std::vector< int >* SuccIDV, std::vector< int >* CostV)
{
    const Hash &sourceHash(idToHash[SourceStateID]);
    XYZNode *sourceNode = sourceHash.node;
    
    ThetaNode *thetaNode = sourceHash.thetaNode;
    
    maps::grid::Index sourceIndex = sourceNode->getIndex();
    
    XYZNode *curNode = sourceNode;
    
    for(const Motion &motion : availableMotions.getMotionForStartTheta(thetaNode->theta))
    {
        TraversabilityGenerator3d::Node *travNode = curNode->getUserData().travNode;
        maps::grid::Index curIndex = curNode->getIndex();
        
        if(!travNode->isExpanded())
        {
            //current node is not drivable
            if(!travGen.expandNode(travNode))
            {
                std::cout << "Node " << travNode->getIndex().transpose() << "Is not drivable" << std::endl;
                return;
            }
        }
        
        int additionalCosts = 0;
        
        bool fail = false;
        for(const Eigen::Vector2i &diff : motion.intermediateCells)
        {
            TraversabilityGenerator3d::Node *newNode = nullptr;
            maps::grid::Index newIndex = curIndex;
            newIndex.x() += diff.x();
            newIndex.y() += diff.y();
            
            //get trav node associated with the next index
            //TODO, this is slow, make it faster
            for(maps::grid::TraversabilityNodeBase *con: travNode->getConnections())
            {
                if(newIndex == con->getIndex())
                {
                    newNode = static_cast<TraversabilityGenerator3d::Node *>(con);
                    break;
                }
            }

            if(!newNode)
            {
                std::cout << "No neighbour node for motion found " << std::endl;
                std::cout << "curIndex " << curIndex.transpose() << " newIndex " << newIndex.transpose() << std::endl;
                fail = true;
                break;
            }
            
            curIndex = newIndex;
            travNode = newNode;

            if(!travNode->isExpanded())
            {
                //current node is not drivable
                if(!travGen.expandNode(travNode))
                {
                    fail = true;
                    break;
                }
            }
            
            //TODO add some collision testing
            
            //TODO add additionalCosts if something is near this node etc
        }

        if(fail)
        {
            std::cout << "Motion passes Node " << travNode->getIndex().transpose() << ", that is not drivable" << std::endl;
            continue;
        }
        
        maps::grid::Index check(sourceIndex);
        check.x() += motion.xDiff;
        check.y() += motion.yDiff;
        
        if(check != curIndex)
        {
            throw std::runtime_error("Error, computation is fishy (Internal error)");
        }
        
        //goal from source to the end of the motion was valid
        
        XYZNode *successXYNode = nullptr;
        ThetaNode *successthetaNode = nullptr;
        const DiscreteTheta curTheta = thetaNode->theta + motion.thetaDiff;
        
        //check if we can connect to the existing graph
        const auto &candidateMap = searchGrid.at(curIndex);
        
        //note, this works, as the equals check is on the height, not the node itself
        auto it = candidateMap.find(curNode);
        
        if(it != candidateMap.end())
        {
            std::cout << "Found existing node " << std::endl;
            //found a node with a matching height
            successXYNode = *it;
        }
        else
        {
            successXYNode = new XYZNode(curNode->getHeight(), curIndex);
            successXYNode->getUserData().travNode = travNode;
        }

        const auto &thetaMap(successXYNode->getUserData().thetaToNodes);
        
        for(const auto &e: thetaMap)
        {
            std::cout << "Elem is " << e.second->id << std::endl;
        }
        
        auto thetaCandidate = thetaMap.find(curTheta);
        if(thetaCandidate != thetaMap.end())
        {
            std::cout << "Found existing State, reconnectiong graph " << std::endl;
            
            successthetaNode = thetaCandidate->second;
        }
        else
        {
            successthetaNode = createNewState(curTheta, successXYNode);
        }
        
        std::cout << "Adding Success Node " << successXYNode->getIndex().transpose() << " trav idx " << travNode->getIndex().transpose() << std::endl;
        
        SuccIDV->push_back(successthetaNode->id);
        CostV->push_back(motion.baseCost + additionalCosts);
    }
}

void EnvironmentXYZTheta::GetPreds(int TargetStateID, std::vector< int >* PredIDV, std::vector< int >* CostV)
{
    SBPL_ERROR("ERROR in EnvNAV2D... function: GetPreds is undefined\n");
    throw new SBPL_Exception();
}

int EnvironmentXYZTheta::SizeofCreatedEnv()
{
    return static_cast<int>(idToHash.size());
}

void EnvironmentXYZTheta::PrintEnv_Config(FILE* fOut)
{

}

void EnvironmentXYZTheta::PrintState(int stateID, bool bVerbose, FILE* fOut)
{
    const Hash &hash(idToHash[stateID]);
    std::cout << "State coordinate " << hash.node->getIndex().transpose() << " " << hash.node->getHeight() << std::endl; //" Theta " << hash.thetaNode->theta << std::endl;
}


