#include "OmniBot.h"

// endian swap
static double byteSwap(double v)
{
    // Only swap if the build target is little endian, cRIO is big endian
    #if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    return v;
    #else

    union {
        uint64_t i;
        double  d;
    } conv;
    conv.d = v;

    conv.i = (conv.i & 0x00000000FFFFFFFF) << 32 | (conv.i & 0xFFFFFFFF00000000) >> 32;
    conv.i = (conv.i & 0x0000FFFF0000FFFF) << 16 | (conv.i & 0xFFFF0000FFFF0000) >> 16;
    conv.i = (conv.i & 0x00FF00FF00FF00FF) << 8  | (conv.i & 0xFF00FF00FF00FF00) >> 8;

    return conv.d;

    #endif
}


OmniBot::OmniBot(const char* address)
    : sender(address), receiver() {

}

RobotResponse OmniBot::setOpenLoop(RobotVelocity velocity) {
    RobotMessage msg;

    msg.type = MSG_OPENLOOP_GOAL;
    velocity.left = byteSwap(velocity.left);
    velocity.right = byteSwap(velocity.right);
    velocity.front = byteSwap(velocity.front);
    velocity.back = byteSwap(velocity.back);

    msg.data.goal = velocity;

    return sender.pushCommand(msg);
}


RobotPose OmniBot::getPose() {
    return receiver.getState().pose;
}

bool OmniBot::stalePose() {
    return receiver.isStale();
}

PIDInfo OmniBot::getPIDInfo() {
    return receiver.getState().control;
}