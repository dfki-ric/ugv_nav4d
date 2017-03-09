#include "PlannerGui.h"
#include <vizkit3d_debug_drawings/DrawingManager.h>

using namespace ::maps::grid;
using namespace base::geometry;

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    vizkit3dDebugDrawings::DrawingManager::disableStandaloneMode();
    PlannerGui gui(argc, argv);
    gui.show();
    app.exec(); 
    return 0;
}
  