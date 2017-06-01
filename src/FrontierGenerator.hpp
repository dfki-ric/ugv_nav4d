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
class EnvironmentXYZTheta;

class FrontierGenerator
{
public:
    
    FrontierGenerator(const maps::grid::TraversabilityMap3d<TravGenNode*>& travMap,
                      const EnvironmentXYZTheta& env);
    
    
    /** Calculate a list of all frontiers that can be visited.
     * @param closeTo used to sort the list. The closer a node is to this position, the closer it is to the begining of the list
     *  List is sorted by TODO*/
    std::vector<base::samples::RigidBodyState> getNextFrontiers(const base::Vector3d& closeTo) const;
    
    
private:
    /** find all frontier nodes*/
    std::vector<const TravGenNode*> getFrontierPatches() const;
    
    /**Figure out goal orientation for each node in @p frontier */
    std::vector<NodeWithOrientation> getFrontierOrientation(const std::vector<const TravGenNode*>& frontier) const;
    
    /**Figure out a node that we can stand on without collisions for each node in @p nodes */
    std::vector<NodeWithOrientation> getCollisionFreeNeighbor(const std::vector<NodeWithOrientation>& nodes) const;
    
    /**Sort nodes according to TODO */
    std::vector<NodeWithOrientation> sortNodes(const std::vector<NodeWithOrientation>& nodes, const base::Vector3d& closeTo) const;
    
    /**convert to rbs */
    std::vector<base::samples::RigidBodyState> getPositions(const std::vector<NodeWithOrientation>& nodes);
    
    const maps::grid::TraversabilityMap3d<TravGenNode*>& travMap;
    const EnvironmentXYZTheta& env;
    
    
};

}