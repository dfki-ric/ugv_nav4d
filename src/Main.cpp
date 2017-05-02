#include <QApplication>
#include "PlannerGui.h"
int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    PlannerGui gui(argc, argv);
    gui.show();
    app.exec(); 
    return 0;
}
  
