#include <iostream>
#include <boost/archive/polymorphic_binary_iarchive.hpp>
#include <motion_planning_libraries/sbpl/SbplMotionPrimitives.hpp>
#include <motion_planning_libraries/sbpl/SbplSplineMotionPrimitives.hpp>
#include <fstream>
#include <backward/backward.hpp>
#include <envire_core/items/Item.hpp>
#include <envire_core/graph/EnvireGraph.hpp>

#include "Planner.hpp"
#include "PlannerGui.h"

#include <base/geometry/Spline.hpp>
#include <iostream>
#include <fstream>

// backward::SignalHandling sh;

using namespace ::maps::grid;
using namespace base::geometry;

int main(int argc, char** argv)
{
    PlannerGui gui(argc, argv);
    gui.exec();
    return 0;
}
