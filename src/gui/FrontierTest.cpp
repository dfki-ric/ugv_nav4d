#include <QApplication>
#include "FrontierTestGui.hpp"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    ugv_nav4d::FrontierTestGui gui(argc, argv);
    gui.show();
    app.exec(); 
    return 0;
}