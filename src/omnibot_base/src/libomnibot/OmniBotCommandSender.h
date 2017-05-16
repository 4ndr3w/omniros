#ifndef OMNIBOT_COMMANDSENDER
#define OMNIBOT_COMMANDSENDER

#include <netinet/in.h>
#include "OmniBotComm.h"

class OmniBotCommandSender {
    int sock;
    sockaddr_in addr;
public:
    OmniBotCommandSender(const char* address);
    RobotResponse pushCommand(RobotMessage command);
};

#endif