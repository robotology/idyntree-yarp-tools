#include "Utilities.h"
#include <cstring>
#include <thread>
#include <csignal>
#include <iostream>

namespace idyntree_yarp_tools {

void my_handler(int sig)
{
    static int ct = 0;

    if (sig == SIGABRT)
    {
        std::cout << "Aborted. " << std::endl;
        if (ct > 3) //to avoid that std::abort is called again
        {
            return;
        }
    }

    ct++;
    if (ct > 3) {
        std::cerr <<  "Aborting (calling abort())..." << std::endl;
        std::abort();
    }
    std::cout << "[try " << ct << " of 3] Trying to shut down." << std::endl;

    isClosing = true;
}

#ifdef WIN32

#include <windows.h>

BOOL WINAPI CtrlHandler(DWORD fdwCtrlType)
{
    switch (fdwCtrlType) {
        // Handle the CTRL-C signal.
    case CTRL_C_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        my_handler(0);
        return TRUE;

    // Handle all other events
    default:
        return FALSE;
    }
}
#endif

void handleSigInt()
{
#ifdef WIN32
    SetConsoleCtrlHandler(CtrlHandler, TRUE);
#else
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = &my_handler;
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGABRT, &action, NULL);
#endif
}

}

