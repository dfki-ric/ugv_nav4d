#include <QApplication>
#include <execinfo.h> //gcc only
#include "PlannerGui.h"


class Application : public QApplication {
public:
    Application(int& argc, char** argv) : QApplication(argc, argv) {}
    
    virtual bool notify(QObject *receiver, QEvent *e) 
    {
        try 
        {
            return QApplication::notify(receiver, e);
        } catch (std::exception &ex) 
        {
            std::cerr << "CAUGHT exception in qt event loop:\n" <<
            ex.what() << std::endl;
            const int traceLen = 40;
            void *symbols[traceLen];
            const size_t size = backtrace(symbols, traceLen);
            backtrace_symbols_fd(symbols, size, STDERR_FILENO);

        } catch (...) {
            std::cerr << "CAUGHT unknown exception in qt event loop" << std::endl;
        }        
         return false;
     }
};

int main(int argc, char** argv)
{
    if(argc < 2)
    {
        std::cout << argv[0] << " [dumpName]" << std::endl;
        return -1;
    }
    
    Application app(argc, argv);
    PlannerGui gui(argv[1]);
    gui.show();
    app.exec(); 
    return 0;
}
  
