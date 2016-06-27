#include <motion_planning_libraries/sbpl/SbplMotionPrimitives.hpp>
#include "EnvironmentXYZTheta.hpp"
#include <sbpl/planners/planner.h>
#include <sbpl/utils/mdpconfig.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <base/Pose.hpp>
#include <fstream>
#include <dwa/SubTrajectory.hpp>

using namespace std;
using namespace motion_planning_libraries;

void EnvironmentXYZTheta::PreComputedMotions::setMotionForTheta(const EnvironmentXYZTheta::Motion& motion, const EnvironmentXYZTheta::DiscreteTheta& theta)
{
    if((int)thetaToMotion.size() <= theta.theta)
    {
        thetaToMotion.resize(theta.theta + 1);
    }
    
    thetaToMotion[theta.theta].push_back(motion);
}


EnvironmentXYZTheta::EnvironmentXYZTheta(boost::shared_ptr< maps::grid::MultiLevelGridMap< maps::grid::SurfacePatchBase > > mlsGrid,
                                         const TraversabilityGenerator3d::Config &travConf,
                                         const motion_planning_libraries::SbplMotionPrimitives& primitives) : 
    travGen(travConf)
    , mlsGrid(mlsGrid)
    , startThetaNode(nullptr)
    , goalThetaNode(nullptr)
    , travConf(travConf)
{
    numAngles = 16;
    
    travGen.setMLSGrid(mlsGrid);
    
    searchGrid.setResolution(Eigen::Vector2d(travConf.gridResolution, travConf.gridResolution));
    searchGrid.extend(travGen.getTraversabilityMap().getNumCells());
    readMotionPrimitives(primitives);
}

EnvironmentXYZTheta::~EnvironmentXYZTheta()
{

}

EnvironmentXYZTheta::ThetaNode* EnvironmentXYZTheta::createNewStateFromPose(const Eigen::Vector3d& pos, double theta, XYZNode **xyzBackNode)
{
    TraversabilityGenerator3d::Node *travNode = travGen.generateStartNode(pos);
    if(!travNode)
    {
        cout << "createNewStateFromPose: Error Pose " << pos.transpose() << " is out of grid" << endl;
        throw runtime_error("Pose is out of grid");
    }
    
    XYZNode *xyzNode = new XYZNode(travNode->getHeight(), travNode->getIndex());
    xyzNode->getUserData().travNode = travNode;
    searchGrid.at(travNode->getIndex()).insert(xyzNode);
    
    DiscreteTheta thetaD(theta, numAngles);
    
    if(xyzBackNode)
        *xyzBackNode = xyzNode;
    
    return createNewState(thetaD, xyzNode);
}


void EnvironmentXYZTheta::setGoal(const Eigen::Vector3d& goalPos, double theta)
{
    goalThetaNode = createNewStateFromPose(goalPos, theta, &goalXYZNode);
}

void EnvironmentXYZTheta::setStart(const Eigen::Vector3d& startPos, double theta)
{
    startThetaNode = createNewStateFromPose(startPos, theta, &startXYZNode);
}

void EnvironmentXYZTheta::SetAllPreds(CMDPSTATE* state)
{
    //implement this if the planner needs access to predecessors

    SBPL_ERROR("ERROR in EnvNAV2D... function: SetAllPreds is undefined\n");
    throw EnvironmentXYZThetaException("SetAllPreds() not implemented");
}

void EnvironmentXYZTheta::SetAllActionsandAllOutcomes(CMDPSTATE* state)
{
    SBPL_ERROR("ERROR in EnvNAV2D... function: SetAllActionsandAllOutcomes is undefined\n");
    throw EnvironmentXYZThetaException("SetAllActionsandAllOutcomes() not implemented");
}

int EnvironmentXYZTheta::GetFromToHeuristic(int FromStateID, int ToStateID)
{
    const Hash &sourceHash(idToHash[FromStateID]);
    XYZNode *sourceNode = sourceHash.node;

    const Hash &targetHash(idToHash[ToStateID]);
    XYZNode *targetNode = targetHash.node;

    //dummy implementation
    return (sourceNode->getIndex() - targetNode->getIndex()).norm();
}

int EnvironmentXYZTheta::GetGoalHeuristic(int stateID)
{
    const Hash &sourceHash(idToHash[stateID]);
    XYZNode *sourceNode = sourceHash.node;

    return (sourceNode->getIndex() - goalXYZNode->getIndex()).norm();
}

int EnvironmentXYZTheta::GetStartHeuristic(int stateID)
{
    const Hash &sourceHash(idToHash[stateID]);
    XYZNode *sourceNode = sourceHash.node;

    return (sourceNode->getIndex() - startXYZNode->getIndex()).norm();
}

bool EnvironmentXYZTheta::InitializeEnv(const char* sEnvFile)
{
    return true;
}

bool EnvironmentXYZTheta::InitializeMDPCfg(MDPConfig* MDPCfg)
{
    if(!goalThetaNode || !startThetaNode)
        return false;
    
    //initialize MDPCfg with the start and goal ids
    MDPCfg->goalstateid = goalThetaNode->id;
    MDPCfg->startstateid = startThetaNode->id;

    return true;
}

EnvironmentXYZTheta::ThetaNode *EnvironmentXYZTheta::createNewState(const DiscreteTheta &curTheta, XYZNode *curNode)
{
    ThetaNode *newNode = new ThetaNode(curTheta);
    newNode->id = idToHash.size();
    Hash hash(curNode, newNode);
    idToHash.push_back(hash);
    curNode->getUserData().thetaToNodes.insert(make_pair(curTheta, newNode));
    
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

TraversabilityGenerator3d::Node *EnvironmentXYZTheta::movementPossible(TraversabilityGenerator3d::Node *fromTravNode, const maps::grid::Index &fromIdx, const maps::grid::Index &toIdx)
{
    TraversabilityGenerator3d::Node *targetNode = nullptr;
    
    if(toIdx == fromIdx)
        return fromTravNode;
    
    //get trav node associated with the next index
    //TODO, this is slow, make it faster
    for(maps::grid::TraversabilityNodeBase *con: fromTravNode->getConnections())
    {
        if(toIdx == con->getIndex())
        {
            targetNode = static_cast<TraversabilityGenerator3d::Node *>(con);
            break;
        }
    }

    if(!targetNode)
    {
        cout << "No neighbour node for motion found " << endl;
        cout << "curIndex " << fromIdx.transpose() << " newIndex " << toIdx.transpose() << endl;
        return nullptr;
    }
    
    if(!targetNode->isExpanded())
    {
        //current node is not drivable
        if(!travGen.expandNode(targetNode))
        {
            return nullptr;
        }
    }
    
    //TODO add some collision testing
    
    //TODO add additionalCosts if something is near this node etc
    
    return targetNode;
}


void EnvironmentXYZTheta::GetSuccs(int SourceStateID, vector< int >* SuccIDV, vector< int >* CostV)
{
    const Hash &sourceHash(idToHash[SourceStateID]);
    XYZNode *sourceNode = sourceHash.node;
    
    ThetaNode *thetaNode = sourceHash.thetaNode;
    
    maps::grid::Index sourceIndex = sourceNode->getIndex();
    
    XYZNode *curNode = sourceNode;

    TraversabilityGenerator3d::Node *travNode = curNode->getUserData().travNode;
    if(!travNode->isExpanded())
    {
        //current node is not drivable
        if(!travGen.expandNode(travNode))
        {
            cout << "Node " << travNode->getIndex().transpose() << "Is not drivable" << endl;
            return;
        }
    }
    
    for(const Motion &motion : availableMotions.getMotionForStartTheta(thetaNode->theta))
    {
        travNode = curNode->getUserData().travNode;
        maps::grid::Index curIndex = curNode->getIndex();
        
        int additionalCosts = 0;
        
        //FIXME intermediateCells has been removed (is not provided by config file)
        for(const base::Pose2D &intermediatePose : motion.intermediatePoses)
        {
            maps::grid::Index newIndex = curIndex;
            maps::grid::Index diff;
            const base::Vector3d position(intermediatePose.position.x(), intermediatePose.position.y(), 0);
            if(!searchGrid.toGrid(position, diff, false))
            {
                std::cout << "Position is " << position.transpose() << std::endl;
                throw EnvironmentXYZThetaException("Cannot convert intermediate Pose to grid cell");
            }
            
            //diff is always a full offset to the start position
            newIndex = sourceIndex + diff;

            
            travNode = movementPossible(travNode, curIndex, newIndex);
            
            if(!travNode)
            {
                break;
            }
            
            curIndex = newIndex;
            
        }
        
        if(!travNode)
        {
            cout << "Motion passes Node " << curIndex.transpose() << ", that is not drivable" << endl;
            continue;
        }
        
        maps::grid::Index finalPos(sourceIndex);
        finalPos.x() += motion.xDiff;
        finalPos.y() += motion.yDiff;
        
        travNode = movementPossible(travNode, curIndex, finalPos);
        if(!travNode)
            continue;
        additionalCosts += (sourceIndex - finalPos).norm() + 1;
        
        curIndex = finalPos;
        
        
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
            cout << "Found existing node " << endl;
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
            cout << "Elem is " << e.second->id << endl;
        }
        
        auto thetaCandidate = thetaMap.find(curTheta);
        if(thetaCandidate != thetaMap.end())
        {
            cout << "Found existing State, reconnectiong graph " << endl;
            
            successthetaNode = thetaCandidate->second;
        }
        else
        {
            successthetaNode = createNewState(curTheta, successXYNode);
        }
        
        cout << "Adding Success Node " << successXYNode->getIndex().transpose() << " trav idx " << travNode->getIndex().transpose() << endl;
        
        SuccIDV->push_back(successthetaNode->id);
        CostV->push_back(motion.baseCost + additionalCosts);
    }
}

void EnvironmentXYZTheta::GetPreds(int TargetStateID, vector< int >* PredIDV, vector< int >* CostV)
{
    SBPL_ERROR("ERROR in EnvNAV2D... function: GetPreds is undefined\n");
    throw EnvironmentXYZThetaException("GetPreds() not implemented");
}

int EnvironmentXYZTheta::SizeofCreatedEnv()
{
    return static_cast<int>(idToHash.size());
}

void EnvironmentXYZTheta::PrintEnv_Config(FILE* fOut)
{
    throw EnvironmentXYZThetaException("PrintEnv_Config() not implemented");
}

void EnvironmentXYZTheta::PrintState(int stateID, bool bVerbose, FILE* fOut)
{
    const Hash &hash(idToHash[stateID]);
    cout << "State coordinate " << hash.node->getIndex().transpose() << " " << hash.node->getHeight() << endl; //" Theta " << hash.thetaNode->theta << endl;
}


void EnvironmentXYZTheta::readVar(const string& varName, int& result, ifstream& file) const
{
    string line;
    vector<string> strs;
    if(!getline(file, line)) 
        throw EnvironmentXYZThetaException("unexpected EOF");
  
    boost::trim(line);
    boost::split(strs, line, boost::is_any_of(":"));
    if(strs.size() != 2 || strs[0] != varName)
        throw EnvironmentXYZThetaException("invalid line: " + line);
    
    boost::trim(strs[1]);
    result = boost::lexical_cast<int>(strs[1]);
}

void EnvironmentXYZTheta::readVar(const string& varName, double& result, ifstream& file) const
{
    string line;
    vector<string> strs;
      if(!getline(file, line)) 
        throw EnvironmentXYZThetaException("unexpected EOF");
  
    boost::trim(line);
    boost::split(strs, line, boost::is_any_of(":"));
    if(strs.size() != 2 || strs[0] != varName)
        throw EnvironmentXYZThetaException("invalid line: " + line);
    
    boost::trim(strs[1]);
    result = boost::lexical_cast<double>(strs[1]);
}

void EnvironmentXYZTheta::readVar(const string& varName, Eigen::Array3i& result, ifstream& file) const
{
    string line;
    vector<string> strs;
    if(!getline(file, line)) 
        throw EnvironmentXYZThetaException("unexpected EOF");
    
    boost::trim(line);
    boost::split(strs, line, boost::is_any_of(": "), boost::token_compress_on);
    
    if(strs.size() != 4 || strs[0] != varName)
        throw EnvironmentXYZThetaException("unabled to parse Pose2D");
    
    boost::trim(strs[1]);
    boost::trim(strs[2]);
    boost::trim(strs[3]);
    result << boost::lexical_cast<int>(strs[1]),
              boost::lexical_cast<int>(strs[2]),                                       
              boost::lexical_cast<int>(strs[3]);
}

void EnvironmentXYZTheta::readPose2D(base::Pose2D& result, std::ifstream& file) const
{
  
    string line;
    if(!getline(file, line)) 
        throw EnvironmentXYZThetaException("unexpected EOF");
  
    vector<string> strs;
    boost::split(strs, line, boost::is_space(), boost::token_compress_on);
    if(strs.size() != 3)
        throw EnvironmentXYZThetaException("unabled to parse Pose2D");
    
    boost::trim(strs[0]);
    boost::trim(strs[1]);
    boost::trim(strs[2]);
    const double x = boost::lexical_cast<double>(strs[0]);
    const double y = boost::lexical_cast<double>(strs[1]);
    const double theta = boost::lexical_cast<double>(strs[2]);
    result = base::Pose2D(base::Position2D(x, y), base::Orientation2D(theta));
}


EnvironmentXYZTheta::Motion EnvironmentXYZTheta::readPrimitive(ifstream& file) const
{
    EnvironmentXYZTheta::Motion result(numAngles);
    int primId = 0;
    int theta = 0;
    Eigen::Array3i discreteEndPose;
    double additionalCostMultplier;
    int numIntermediatePoses;
    
    SBPL_DEBUG("===== Reading Primitive =====\n");
    readVar("primID", primId, file);
    readVar("startangle_c", theta, file);
    readVar("endpose_c", discreteEndPose, file);
    readVar("additionalactioncostmult", additionalCostMultplier, file);
    readVar("intermediateposes", numIntermediatePoses, file);
    result.intermediatePoses.resize(numIntermediatePoses);
    
    SBPL_DEBUG("primID = %d\n", primId);
    SBPL_DEBUG("theta = %d\n", theta);
    SBPL_DEBUG("discreteEndPose = %d, %d, %d\n", discreteEndPose[0], discreteEndPose[1], discreteEndPose[2]);
    SBPL_DEBUG("additionalCostMultplier = %f\n", additionalCostMultplier);
    SBPL_DEBUG("numIntermediatePoses = %d\n", numIntermediatePoses);
    
    SBPL_DEBUG("Intermediate Poses:\n");
    for(int i = 0; i < numIntermediatePoses; ++i)
    {
        readPose2D(result.intermediatePoses[i], file);
        SBPL_DEBUG("%f, %f, %f\n", pose.position.x(), pose.position.y(), pose.orientation);
    }
    result.startTheta = DiscreteTheta(theta, numAngles);
    result.xDiff = discreteEndPose[0];
    result.yDiff = discreteEndPose[1];
    result.thetaDiff = DiscreteTheta(discreteEndPose[2] - theta, numAngles);
    return result;
}


void EnvironmentXYZTheta::readMotionPrimitives(const string& path)
{
  
    /* File content example:
    resolution_m: 0.100000
    numberofangles: 16
    totalnumberofprimitives: 192
    primID: 0
    startangle_c: 0
    endpose_c: 10 0 0
    additionalactioncostmult: 1
    intermediateposes: 10
    0.0000 0.0000 0.0000
    0.1111 0.0000 0.0000
    0.2222 0.0000 0.0000
    0.3333 0.0000 0.0000
    0.4444 0.0000 0.0000
    0.5556 0.0000 0.0000
    0.6667 0.0000 0.0000
    0.7778 0.0000 0.0000
    0.8889 0.0000 0.0000
    1.0000 0.0000 0.0000 */
  
    SBPL_DEBUG("Reading motion primitives from: %s\n", path.c_str());
    ifstream file(path);
    if(!file.is_open())
        throw EnvironmentXYZThetaException("Unable to open motion primitive file " + path);
    
    //read header
    double resolution = 0;;
    int numAngles = 0;
    int totalNumPrimitives = 0;
    readVar("resolution_m", resolution, file);
    readVar("numberofangles", numAngles, file);
    readVar("totalnumberofprimitives", totalNumPrimitives, file);
    
    SBPL_DEBUG("resolution = %f\n", resolution);
    SBPL_DEBUG("numAngles = %d\n", numAngles);    
    SBPL_DEBUG("totalnumberofprimitives = %d\n", totalNumPrimitives);
    
    if(std::abs(travConf.gridResolution - resolution) > 0.000001)
        throw EnvironmentXYZThetaException("grid resolution missmatch. Expected: " + 
                                           boost::lexical_cast<string>(travConf.gridResolution) +
                                           " but got: " + 
                                           boost::lexical_cast<string>(resolution));
    
    //read primitives
    for(int i = 0; i < totalNumPrimitives; ++i)
    {
        const Motion motion = readPrimitive(file);
        std::cout << "Adding Motion: 0, 0, " << motion.startTheta.getTheta() << " -> " << motion.xDiff << ", " << motion.yDiff << ", " << motion.thetaDiff.getTheta() << std::endl;
        availableMotions.setMotionForTheta(motion, motion.startTheta);
    }
}

void EnvironmentXYZTheta::readMotionPrimitives(const SbplMotionPrimitives& primitives)
{
  
    for(const Primitive& prim : primitives.mListPrimitives)
    {
      Motion motion(numAngles);
      std::cout << "---------------------------------------------" << std::endl;
      
      motion.xDiff = prim.mEndPose[0];
      motion.yDiff = prim.mEndPose[1];
      motion.thetaDiff =  DiscreteTheta(prim.mStartAngle - static_cast<int>(prim.mEndPose[2]), numAngles);
      motion.startTheta = DiscreteTheta(prim.mStartAngle, numAngles);
      std::cout << "Adding Motion: 0, 0, " << motion.startTheta.getTheta() << " -> " << motion.xDiff << ", " << motion.yDiff << ", " << motion.thetaDiff.getTheta() << std::endl;
      
      std::vector<base::Pose2D> poses;
      bool allPositionsSame = true;
      base::Vector2d firstPos = prim.mIntermediatePoses.front().topRows(2);
      for(const base::Vector3d& pose : prim.mIntermediatePoses)
      {
          poses.emplace_back(base::Position2D(pose[0], pose[1]), pose[2]);
          if(pose.topRows(2) != firstPos)
          {
              allPositionsSame = false;
          }
      }
      
      if(!allPositionsSame)
      {
          //fit a spline through the primitive's intermediate points and use that spline to
          //create intermediate points for the motion.
          //This is done to ensure that every cell is hit by one intermediate point.
          //if the positions are different, normal spline interpolation works
          SubTrajectory spline;
          spline.interpolate(poses);
          const double splineLength = spline.getDistToGoal(spline.getStartParam());
          //set stepDist so small that we are guaranteed to oversample
          const double stepDist = travConf.gridResolution - (travConf.gridResolution / 2.0);
          double currentDist = 0;
          double currentParam = spline.getStartParam();
          while(currentDist < splineLength)
          {
            currentDist += stepDist;
            currentParam = spline.advance(currentParam, stepDist);
            const base::Pose2D currentPose = spline.getIntermediatePointNormalized(currentParam);
            motion.intermediatePoses.push_back(currentPose);
          }
      }
      else
      {
        //Since our index is only x/y based (i.e. it ignores rotation)
        //we do not need to add any intermediate positions for rotation-only
        //movements.
      }
      availableMotions.setMotionForTheta(motion, motion.startTheta);
    }
}



