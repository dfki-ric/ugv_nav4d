#include <QApplication>
#include <execinfo.h> //gcc only
#include "PlannerGui.h"
#include <base-logging/Logging.hpp>


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
            LOG_ERROR_S << "CAUGHT exception in qt event loop:\n" << ex.what();
            const int traceLen = 40;
            void *symbols[traceLen];
            const size_t size = backtrace(symbols, traceLen);
            backtrace_symbols_fd(symbols, size, STDERR_FILENO);

        } catch (...) {
            LOG_ERROR_S << "CAUGHT unknown exception in qt event loop";
        }        
         return false;
     }
};

int main(int argc, char** argv)
{
    // Same GUI-process tuning as the travgen debug GUI: OpenMP workers must not
    // spin against the llvmpipe software-render threads at every wave barrier,
    // and llvmpipe does not need one thread per logical CPU for a debug view.
    // overwrite=0 keeps user overrides working.
    setenv("OMP_WAIT_POLICY", "passive", 0);
    setenv("LP_NUM_THREADS", "4", 0);

    Application app(argc, argv);
    PlannerGui gui(argc, argv);
    gui.show();
    app.exec(); 
    return 0;
}
  
