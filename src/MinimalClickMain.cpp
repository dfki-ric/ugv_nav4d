#include "MinimalClickGui.hpp"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    MinimalClickGui gui(argc, argv);
    gui.show();
    app.exec(); 
    return 0;
}