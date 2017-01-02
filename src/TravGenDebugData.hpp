#pragma once
#include "UgvDebug.hpp"
#include "TraversabilityConfig.hpp"
#include "TravGenNode.hpp"
#include <vector>

namespace ugv_nav4d 
{
    class TraversabilityGenerator3d;
}

namespace ugv_nav4d_debug
{

    class TravGenDebugData
    {
    public:
        
        TravGenDebugData() : travGen(nullptr) {}
        
        void setTravConfig(const ugv_nav4d::TraversabilityConfig& conf)
        {
            travConf = conf;
        }
        
        void setTravGen(ugv_nav4d::TraversabilityGenerator3d* gen);
        
        void planeComputed(const ugv_nav4d::TravGenNode& node);
        
        const std::vector<Eigen::Vector4d>& getSlopes()
        {
            return slopes;
        }
        
        const std::vector<Eigen::Matrix<double, 2, 3>>& getSlopeDirs()
        {
            return slopeDirections;
        }

    private:
        
        //[0..2] = coordinate, [3] = slope
        std::vector<Eigen::Vector4d> slopes;
        //row(0) = position, row(1) = direction vector
        std::vector<Eigen::Matrix<double, 2, 3>> slopeDirections;
        ugv_nav4d::TraversabilityConfig travConf;
        ugv_nav4d::TraversabilityGenerator3d* travGen;
    };
}

//     /**Contains the slopes of all travnodes if debug is defined */
//     mutable std::vector<Eigen::Vector4d> debugSlopes;
//     /**Contains the sloep directions of all travnoces if debug is defined.
//      * rows(0) is the location, rows(1) the slope direction*/
//     mutable std::vector<Eigen::Matrix<double, 2, 3>> debugSlopeDirs;
