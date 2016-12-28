#pragma once

namespace ugv_nav4d
{

class TraversabilityConfig
{
public:
    TraversabilityConfig(): maxStepHeight(0), maxSlope(0), robotHeight(0), robotSizeX(0),    gridResolution(0.0) {};
    
    double maxStepHeight;
    double maxSlope;
    double robotHeight;
    double robotSizeX;
    double robotSizeY;
    double slopeMetricScale;
    double gridResolution;
};

}