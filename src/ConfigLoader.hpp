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
            
            YAML::Node params = config;
            if (config["ugv_nav4d_ros2"] && config["ugv_nav4d_ros2"]["ros__parameters"]) {
                params = config["ugv_nav4d_ros2"]["ros__parameters"];
            }

            // Load spline config
            if (params["splineConfig"]) {
                auto sc = params["splineConfig"];
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
            } else {
                double res = params["grid_resolution"] ? params["grid_resolution"].as<double>(0.5) : (params["gridSize"] ? params["gridSize"].as<double>(0.5) : 0.5);
                splineConfig.gridSize = res;
                splineConfig.numAngles = params["numAngles"] ? params["numAngles"].as<unsigned>(16) : 16;
                splineConfig.numEndAngles = params["numEndAngles"] ? params["numEndAngles"].as<unsigned>(8) : 8;
                splineConfig.destinationCircleRadius = params["destinationCircleRadius"] ? params["destinationCircleRadius"].as<double>(6) : 6;
                splineConfig.cellSkipFactor = params["cellSkipFactor"] ? params["cellSkipFactor"].as<double>(0.1) : 0.1;
                splineConfig.generatePointTurnMotions = params["generatePointTurnMotions"] ? params["generatePointTurnMotions"].as<bool>(true) : true;
                splineConfig.generateLateralMotions = params["generateLateralMotions"] ? params["generateLateralMotions"].as<bool>(true) : true;
                splineConfig.generateBackwardMotions = params["generateBackwardMotions"] ? params["generateBackwardMotions"].as<bool>(true) : true;
                splineConfig.generateForwardMotions = params["generateForwardMotions"] ? params["generateForwardMotions"].as<bool>(true) : true;
                splineConfig.splineOrder = params["splineOrder"] ? params["splineOrder"].as<unsigned>(4) : 4;
            }
            
            // Load mobility config
            if (params["mobilityConfig"]) {
                auto mc = params["mobilityConfig"];
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
                mobilityConfig.curvaturePenaltyWeight = mc["curvaturePenaltyWeight"] ? mc["curvaturePenaltyWeight"].as<double>(0.0) : 0.0;
                mobilityConfig.angularCostWeight = mc["angularCostWeight"] ? mc["angularCostWeight"].as<double>(1.0) : 1.0;
            } else {
                mobilityConfig.translationSpeed = params["translationSpeed"] ? params["translationSpeed"].as<double>(0.5) : 0.5;
                mobilityConfig.rotationSpeed = params["rotationSpeed"] ? params["rotationSpeed"].as<double>(0.5) : 0.5;
                mobilityConfig.minTurningRadius = params["minTurningRadius"] ? params["minTurningRadius"].as<double>(1.0) : 1.0;
                mobilityConfig.searchRadius = params["searchRadius"] ? params["searchRadius"].as<double>(0.0) : 0.0;
                mobilityConfig.searchProgressSteps = params["searchProgressSteps"] ? params["searchProgressSteps"].as<double>(0.1) : 0.1;
                mobilityConfig.multiplierForward = params["multiplierForward"] ? params["multiplierForward"].as<double>(1.0) : 1.0;
                mobilityConfig.multiplierForwardTurn = params["multiplierForwardTurn"] ? params["multiplierForwardTurn"].as<double>(2.0) : 2.0;
                mobilityConfig.multiplierBackward = params["multiplierBackward"] ? params["multiplierBackward"].as<double>(2.0) : 2.0;
                mobilityConfig.multiplierBackwardTurn = params["multiplierBackwardTurn"] ? params["multiplierBackwardTurn"].as<double>(3.0) : 3.0;
                mobilityConfig.multiplierLateral = params["multiplierLateral"] ? params["multiplierLateral"].as<double>(4.0) : 4.0;
                mobilityConfig.multiplierLateralCurve = params["multiplierLateralCurve"] ? params["multiplierLateralCurve"].as<double>(4.0) : 4.0;
                mobilityConfig.multiplierPointTurn = params["multiplierPointTurn"] ? params["multiplierPointTurn"].as<double>(3.0) : 3.0;
                mobilityConfig.maxMotionCurveLength = params["maxMotionCurveLength"] ? params["maxMotionCurveLength"].as<double>(100.0) : 100.0;
                mobilityConfig.spline_sampling_resolution = params["spline_sampling_resolution"] ? params["spline_sampling_resolution"].as<double>(0.05) : 0.05;
                mobilityConfig.remove_goal_offset = params["remove_goal_offset"] ? params["remove_goal_offset"].as<bool>(false) : false;
                mobilityConfig.curvaturePenaltyWeight = params["curvaturePenaltyWeight"] ? params["curvaturePenaltyWeight"].as<double>(0.0) : 0.0;
                mobilityConfig.angularCostWeight = params["angularCostWeight"] ? params["angularCostWeight"].as<double>(1.0) : 1.0;
            }
            
            // Load traversability config
            if (params["travConfig"]) {
                auto tc = params["travConfig"];
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
                travConfig.obstacleInflationMultiplier = tc["obstacleInflationMultiplier"] ? tc["obstacleInflationMultiplier"].as<double>(1.0) : 1.0;
                travConfig.partiallyTraversableMultiplier = tc["partiallyTraversableMultiplier"] ? tc["partiallyTraversableMultiplier"].as<double>(2.0) : 2.0;

                
                std::string slopeMetricStr = tc["slopeMetric"].as<std::string>("NONE");
                if (slopeMetricStr == "AVG_SLOPE") travConfig.slopeMetric = traversability_generator3d::SlopeMetric::AVG_SLOPE;
                else if (slopeMetricStr == "MAX_SLOPE") travConfig.slopeMetric = traversability_generator3d::SlopeMetric::MAX_SLOPE;
                else if (slopeMetricStr == "TRIANGLE_SLOPE") travConfig.slopeMetric = traversability_generator3d::SlopeMetric::TRIANGLE_SLOPE;
                else travConfig.slopeMetric = traversability_generator3d::SlopeMetric::NONE;
            } else {
                travConfig.gridResolution = params["grid_resolution"] ? params["grid_resolution"].as<double>(0.3) : (params["gridResolution"] ? params["gridResolution"].as<double>(0.3) : 0.3);
                travConfig.maxSlope = params["maxSlope"] ? params["maxSlope"].as<double>(0.45) : 0.45;
                travConfig.maxStepHeight = params["maxStepHeight"] ? params["maxStepHeight"].as<double>(0.25) : 0.25;
                travConfig.robotSizeX = params["robotSizeX"] ? params["robotSizeX"].as<double>(0.5) : 0.5;
                travConfig.robotSizeY = params["robotSizeY"] ? params["robotSizeY"].as<double>(0.5) : 0.5;
                travConfig.robotHeight = params["robotHeight"] ? params["robotHeight"].as<double>(0.5) : 0.5;
                travConfig.slopeMetricScale = params["slopeMetricScale"] ? params["slopeMetricScale"].as<double>(1.0) : 1.0;
                travConfig.inclineLimittingMinSlope = params["inclineLimittingMinSlope"] ? params["inclineLimittingMinSlope"].as<double>(0.22) : 0.22;
                travConfig.inclineLimittingLimit = params["inclineLimittingLimit"] ? params["inclineLimittingLimit"].as<double>(0.43) : 0.43;
                travConfig.costFunctionDist = params["costFunctionDist"] ? params["costFunctionDist"].as<double>(0.0) : 0.0;
                travConfig.distToGround = params["distToGround"] ? params["distToGround"].as<double>(0.0) : 0.0;
                travConfig.minTraversablePercentage = params["minTraversablePercentage"] ? params["minTraversablePercentage"].as<double>(0.5) : 0.5;
                travConfig.allowForwardDownhill = params["allowForwardDownhill"] ? params["allowForwardDownhill"].as<bool>(true) : true;
                travConfig.enableInclineLimitting = params["enableInclineLimitting"] ? params["enableInclineLimitting"].as<bool>(false) : false;
                travConfig.obstacleInflationMultiplier = params["obstacleInflationMultiplier"] ? params["obstacleInflationMultiplier"].as<double>(1.0) : 1.0;
                travConfig.partiallyTraversableMultiplier = params["partiallyTraversableMultiplier"] ? params["partiallyTraversableMultiplier"].as<double>(2.0) : 2.0;

                
                std::string slopeMetricStr = "NONE";
                if (params["slopeMetric"]) {
                    slopeMetricStr = params["slopeMetric"].as<std::string>("NONE");
                    if (slopeMetricStr.length() > 0 && slopeMetricStr[0] == ':') {
                        slopeMetricStr = slopeMetricStr.substr(1);
                    }
                }
                if (slopeMetricStr == "AVG_SLOPE") travConfig.slopeMetric = traversability_generator3d::SlopeMetric::AVG_SLOPE;
                else if (slopeMetricStr == "MAX_SLOPE") travConfig.slopeMetric = traversability_generator3d::SlopeMetric::MAX_SLOPE;
                else if (slopeMetricStr == "TRIANGLE_SLOPE") travConfig.slopeMetric = traversability_generator3d::SlopeMetric::TRIANGLE_SLOPE;
                else travConfig.slopeMetric = traversability_generator3d::SlopeMetric::NONE;
            }
            
            // Load planner config
            if (params["plannerConfig"]) {
                auto pc = params["plannerConfig"];
                plannerConfig.epsilonSteps = pc["epsilonSteps"].as<double>(2.0);
                plannerConfig.initialEpsilon = pc["initialEpsilon"].as<double>(64.0);
                plannerConfig.numThreads = pc["numThreads"].as<unsigned>(4);
                plannerConfig.usePathStatistics = pc["usePathStatistics"].as<bool>(false);
                plannerConfig.searchUntilFirstSolution = pc["searchUntilFirstSolution"].as<bool>(false);
            } else {
                plannerConfig.epsilonSteps = params["epsilonSteps"] ? params["epsilonSteps"].as<double>(2.0) : 2.0;
                plannerConfig.initialEpsilon = params["initialEpsilon"] ? params["initialEpsilon"].as<double>(64.0) : 64.0;
                plannerConfig.numThreads = params["numThreads"] ? params["numThreads"].as<unsigned>(4) : 4;
                plannerConfig.usePathStatistics = params["usePathStatistics"] ? params["usePathStatistics"].as<bool>(false) : false;
                plannerConfig.searchUntilFirstSolution = params["searchUntilFirstSolution"] ? params["searchUntilFirstSolution"].as<bool>(false) : false;
            }
            
            return true;
        } catch (const std::exception& e) {
            LOG_ERROR_S << "Failed to load config: " << e.what();
            return false;
        }
    }
};

}
