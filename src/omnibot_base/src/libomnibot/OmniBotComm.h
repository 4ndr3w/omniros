#ifndef OMNIBOT_COMM
#define OMNIBOT_COMM

#include <stdint.h>
struct RobotVelocity {
    double front;
    double back;
    double left;
    double right;
};

struct RobotPose {
    double x;
    double y;
    double theta;
    
    double vx;
    double vy;
    double vth;
    
    double forwardDistance;
};

struct PIDInfo {
    double frontSetpoint;
    double frontActual;

    double backSetpoint;
    double backActual;

    double leftSetpoint;
    double leftActual;

    double rightSetpoint;
    double rightActual;
};

struct RobotState {
    uint8_t mode;
    
    RobotPose pose;
    PIDInfo control;
};

#define MSG_SET_PID_FRONT 1
#define MSG_SET_PID_BACK 2
#define MSG_SET_PID_LEFT 3
#define MSG_SET_PID_RIGHT 4
struct PIDConstants {
    double p;
    double i;
    double d;
    double f;
};

#define MSG_RESET_POSE 5

#define MSG_VELOCITY_GOAL 6
#define MSG_OPENLOOP_GOAL 7

struct RobotMessage {
    uint8_t type;
    union {
        RobotVelocity goal; 
        PIDConstants pid;
    } data;
};

struct RobotResponse {
    bool result;
};



#endif
