#pragma once
#include "Planner.hpp"
#include <base/samples/RigidBodyState.hpp>
#include <maps/grid/TraversabilityMap3d.hpp>
#include "TravGenNode.hpp"

/* Konzept
 * - Eine Liste mit bewerteten frontiers rausgeben.
 * - sortiert nach einer bewertungsfunktion
 * - nicht die frontiers selber rausgeben sondern jeweils den nächsten erreichbaren patch auf dem der roboter stehen kann
 * - TODO wie behandlet man die ziel orientierung?
 *    - Man gibt nur patches raus wo der bot in jeder orientierung stehen kann? Das geht schief bei hängen
 *    - Man gibt für den patch eine liste von erlaubten orientierungen mit raus
 *    - Der User muss die zielorientierung mit angeben und hat dann halt pech wenns nicht klappt?
 * 
 * 
 * - Karte vom Bunker mal laden zum testen
 * 
 *  Stefan Hase
 * Altes exploration modul: https://git.hb.dfki.de/dfki-planning/exploration
 *                          + orogen_exploration
 * 
 * */


namespace ugv_nav4d 
{
struct NodeWithOrientation;
struct NodeWithOrientationAndCost;
class EnvironmentXYZTheta;

class FrontierGenerator
{
    
public:
    
    FrontierGenerator(const TraversabilityConfig& travConf);
    
    template <maps::grid::MLSConfig::update_model SurfacePatch>
    void updateMap(const maps::grid::MLSMap<SurfacePatch>& mls)
    {
        mlsMap.reset(new TraversabilityGenerator3d::MLGrid(mls));
        travGen.setMLSGrid(mlsMap);
    }
    
    void updateRobotPos(const base::Vector3d& robotPos);
    
    void updateGoalPos(const base::Vector3d& goalPos);
    
    
    /** Calculate a list of all frontiers that can be visited.
     * @param closeTo used to sort the list. The closer a node is to this position, the closer it is to the begining of the list
     *  List is sorted by TODO*/
    std::vector<base::samples::RigidBodyState> getNextFrontiers(const base::Vector3d& closeTo);
    
    
private:

    /** find all frontier nodes*/
    std::vector<const TravGenNode*> getFrontierPatches() const;
    
    /**Figure out goal orientation for each node in @p frontier */
    std::vector<NodeWithOrientation> getFrontierOrientation(const std::vector<const TravGenNode*>& frontier) const;
    
    /**Figure out a node that we can stand on without collisions for each node in @p nodes */
    std::vector<NodeWithOrientation> getCollisionFreeNeighbor(const std::vector<NodeWithOrientation>& nodes) const;
    
    /** TODO describe what cost contains and what is a good/bad value */
    std::vector<NodeWithOrientationAndCost> calculateCost(const TravGenNode* startNode,
                                                          const base::Vector3d& goalPos,
                                                          const std::vector<NodeWithOrientation>& nodes) const;
    
    /**Sort nodes according to TODO */
    std::vector<NodeWithOrientation> sortNodes(const std::vector<NodeWithOrientation>& nodes, const base::Vector3d& closeTo) const;
    
    /**convert to rbs */
    std::vector<base::samples::RigidBodyState> getPositions(const std::vector<NodeWithOrientation>& nodes) const;
    
    /** Calculate the number of patches that can still be explored in the vicinity
     * of @p node.
     * The number is normalized to [0..1]
     * 1 means fully explored, 0 means not explored at all*/
    double calcExplorablePatches(const TravGenNode* node) const;
    
    /**Returns the distance from @p node to the ray starting from @p rayOrigin going through @p rayThrough */
    double distToRay(const TravGenNode* node, const base::Vector3d& rayOrigin, const base::Vector3d& rayThrough) const;
    
    base::Vector3d nodeCenterPos(const TravGenNode* node) const;

private:
    
    TraversabilityConfig travConf;
    TraversabilityGenerator3d travGen;
    boost::shared_ptr<TraversabilityGenerator3d::MLGrid> mlsMap;
    base::Vector3d robotPos;
    base::Vector3d goalPos;//FIXME bad name, this is the point somewhere in the distance that indicates the direction of exploration
};


}