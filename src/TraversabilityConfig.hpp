#pragma once

namespace ugv_nav4d
{

class TraversabilityConfig
{
public:
    TraversabilityConfig(): maxStepHeight(0), maxSlope(0), robotHeight(0), robotSizeX(0), maxGapSize(0), numTraversabilityClasses(0), numNominalMeasurements(1), outliertFilterMinMeasurements(0), outliertFilterMaxStdDev(0.0), gridResolution(0.0) {};
    double maxStepHeight;
    double maxSlope;
    double robotHeight;
    double robotSizeX;
    double robotSizeY;
    double maxGapSize;
    int numTraversabilityClasses;
    /**
        * The amount of measurements a MLS-Patch needs
        * to get a probability of 1.0
        * */
    int numNominalMeasurements;
    
    int outliertFilterMinMeasurements;
    double outliertFilterMaxStdDev;
    double gridResolution;
};

}