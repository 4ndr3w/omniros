#ifndef OMNIBOT_STATERECEIVER
#define OMNIBOT_STATERECEIVER

#include "OmniBotComm.h"
#include <thread>
#include <atomic>

class OmniBotStateReceiver {
    std::thread thread;
    std::atomic<RobotState> state;
    bool stale;
    bool running;
    int sock;
    void socketThread();
public:
    OmniBotStateReceiver();
    ~OmniBotStateReceiver();
    RobotState getState();
    bool isStale();
};

#endif