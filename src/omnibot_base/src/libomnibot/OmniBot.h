#ifndef OMNIBOT_H
#define OMNIBOT_H

#include "OmniBotComm.h"
#include "OmniBotCommandSender.h"
#include "OmniBotStateReceiver.h"

class OmniBot {
    OmniBotCommandSender sender;
    OmniBotStateReceiver receiver;
public:
    OmniBot(const char* address);

    RobotResponse setOpenLoop(RobotVelocity velocity);

    bool stalePose();
    RobotPose getPose();
    PIDInfo getPIDInfo();
};

#endif