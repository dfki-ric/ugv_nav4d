#pragma once

#include <string>
#include <yaml-cpp/yaml.h>
#include <ugv_nav4d/Planner.hpp>
#include <traversability_generator3d/TraversabilityGenerator3d.hpp>
#include <sbpl_spline_primitives/SbplSplineMotionPrimitives.hpp>

namespace ugv_nav4d {

class ConfigLoader {
public:
    static bool loadConfig(
        const std::string& configPath,
        sbpl_spline_primitives::SplinePrimitivesConfig& splineConfig,
        ugv_nav4d::Mobility& mobilityConfig,
        traversability_generator3d::TraversabilityConfig& travConfig,
        ugv_nav4d::PlannerConfig& plannerConfig)
    {
        try {
            YAML::Node config = YAML::LoadFile(configPath);
            
            // Load spline config
            if (config["splineConfig"]) {
                auto sc = config["splineConfig"];
                splineConfig.gridSize = sc["gridSize"].as<double>(0.5);
                splineConfig.numAngles = sc["numAngles"].as<unsigned>(16);
                splineConfig.numEndAngles = sc["numEndAngles"].as<unsigned>(8);
                splineConfig.destinationCircleRadius = sc["destinationCircleRadius"].as<double>(6);
                splineConfig.cellSkipFactor = sc["cellSkipFactor"].as<double>(0.1);
                splineConfig.generatePointTurnMotions = sc["generatePointTurnMotions"].as<bool>(true);
                splineConfig.generateLateralMotions = sc["generateLateralMotions"].as<bool>(true);
                splineConfig.generateBackwardMotions = sc["generateBackwardMotions"].as<bool>(true);
                splineConfig.generateForwardMotions = sc["generateForwardMotions"].as<bool>(true);
                splineConfig.splineOrder = sc["splineOrder"].as<unsigned>(4);
            }
            
            // Load mobility config
            if (config["mobilityConfig"]) {
                auto mc = config["mobilityConfig"];
                mobilityConfig.translationSpeed = mc["translationSpeed"].as<double>(0.5);
                mobilityConfig.rotationSpeed = mc["rotationSpeed"].as<double>(0.5);
                mobilityConfig.minTurningRadius = mc["minTurningRadius"].as<double>(1);
                mobilityConfig.searchRadius = mc["searchRadius"].as<double>(0.0);
                mobilityConfig.searchProgressSteps = mc["searchProgressSteps"].as<double>(0.1);
                mobilityConfig.multiplierForward = mc["multiplierForward"].as<double>(1);
                mobilityConfig.multiplierForwardTurn = mc["multiplierForwardTurn"].as<double>(2);
                mobilityConfig.multiplierBackward = mc["multiplierBackward"].as<double>(2);
                mobilityConfig.multiplierBackwardTurn = mc["multiplierBackwardTurn"].as<double>(3);
                mobilityConfig.multiplierLateral = mc["multiplierLateral"].as<double>(4);
                mobilityConfig.multiplierLateralCurve = mc["multiplierLateralCurve"].as<double>(4);
                mobilityConfig.multiplierPointTurn = mc["multiplierPointTurn"].as<double>(3);
                mobilityConfig.maxMotionCurveLength = mc["maxMotionCurveLength"].as<double>(100);
                mobilityConfig.spline_sampling_resolution = mc["spline_sampling_resolution"].as<double>(0.05);
                mobilityConfig.remove_goal_offset = mc["remove_goal_offset"].as<bool>(false);
            }
            
            // Load traversability config
            if (config["travConfig"]) {
                auto tc = config["travConfig"];
                travConfig.gridResolution = tc["gridResolution"].as<double>(0.3);
                travConfig.maxSlope = tc["maxSlope"].as<double>(0.45);
                travConfig.maxStepHeight = tc["maxStepHeight"].as<double>(0.25);
                travConfig.robotSizeX = tc["robotSizeX"].as<double>(0.5);
                travConfig.robotSizeY = tc["robotSizeY"].as<double>(0.5);
                travConfig.robotHeight = tc["robotHeight"].as<double>(0.5);
                travConfig.slopeMetricScale = tc["slopeMetricScale"].as<double>(1.0);
                travConfig.inclineLimittingMinSlope = tc["inclineLimittingMinSlope"].as<double>(0.22);
                travConfig.inclineLimittingLimit = tc["inclineLimittingLimit"].as<double>(0.43);
                travConfig.costFunctionDist = tc["costFunctionDist"].as<double>(0.0);
                travConfig.distToGround = tc["distToGround"].as<double>(0.0);
                travConfig.minTraversablePercentage = tc["minTraversablePercentage"].as<double>(0.5);
                travConfig.allowForwardDownhill = tc["allowForwardDownhill"].as<bool>(true);
                travConfig.enableInclineLimitting = tc["enableInclineLimitting"].as<bool>(false);
                
                std::string slopeMetricStr = tc["slopeMetric"].as<std::string>("NONE");
                if (slopeMetricStr == "AVG_SLOPE") travConfig.slopeMetric = traversability_generator3d::SlopeMetric::AVG_SLOPE;
                else if (slopeMetricStr == "MAX_SLOPE") travConfig.slopeMetric = traversability_generator3d::SlopeMetric::MAX_SLOPE;
                else if (slopeMetricStr == "TRIANGLE_SLOPE") travConfig.slopeMetric = traversability_generator3d::SlopeMetric::TRIANGLE_SLOPE;
                else travConfig.slopeMetric = traversability_generator3d::SlopeMetric::NONE;
            }
            
            // Load planner config
            if (config["plannerConfig"]) {
                auto pc = config["plannerConfig"];
                plannerConfig.epsilonSteps = pc["epsilonSteps"].as<double>(2.0);
                plannerConfig.initialEpsilon = pc["initialEpsilon"].as<double>(64.0);
                plannerConfig.numThreads = pc["numThreads"].as<unsigned>(4);
                plannerConfig.usePathStatistics = pc["usePathStatistics"].as<bool>(false);
                plannerConfig.searchUntilFirstSolution = pc["searchUntilFirstSolution"].as<bool>(false);
            }
            
            return true;
        } catch (const std::exception& e) {
            LOG_ERROR_S << "Failed to load config: " << e.what();
            return false;
        }
    }
};

}
