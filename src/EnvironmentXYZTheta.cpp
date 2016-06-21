#include "EnvironmentXYZTheta.hpp"
#include <sbpl/planners/planner.h>

EnvironmentXYZTheta::EnvironmentXYZTheta(boost::shared_ptr< maps::grid::MultiLevelGridMap< maps::grid::SurfacePatchBase > > mlsGrid) : travGen(TraversabilityGenerator3d::Config()), mlsGrid(mlsGrid)
{
}

EnvironmentXYZTheta::~EnvironmentXYZTheta()
{

}


void EnvironmentXYZTheta::SetAllPreds(CMDPSTATE* state)
{
    //implement this if the planner needs access to predecessors

    SBPL_ERROR("ERROR in EnvNAV2D... function: SetAllPreds is undefined\n");
    throw new SBPL_Exception();
}

void EnvironmentXYZTheta::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{

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
    return true;
}

EnvironmentXYZTheta::ThetaNode *EnvironmentXYZTheta::createNewState(const DiscreteTheta &curTheta, XYZNode *curNode)
{
    ThetaNode *newNode = new ThetaNode(curTheta);
    newNode->id = idToHash.size();
    Hash hash(curNode, newNode);
    idToHash.push_back(hash);
    
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
                return;
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
            continue;
        
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
            //found a node with a matching height
            successXYNode = *it;
        }
        else
        {
            successXYNode = new XYZNode(curNode->getHeight(), curIndex);
        }

        const auto &thetaMap(successXYNode->getUserData().thetaToNodes);
        auto thetaCandidate = thetaMap.find(curTheta);
        if(thetaCandidate != thetaMap.end())
        {
            successthetaNode = thetaCandidate->second;
        }
        else
        {
            successthetaNode = createNewState(curTheta, successXYNode);            
            successXYNode->getUserData().thetaToNodes.insert(std::make_pair(curTheta, successthetaNode));
        }
        
        SuccIDV->push_back(successthetaNode->id);
        CostV->push_back(motion.baseCost + additionalCosts);
    }
}
