#ifndef OMNIBOT_COMM
#define OMNIBOT_COMM
#include <stdint.h>

struct RobotCommand {
  float leftVelocity;
  float rightVelocity;
  float frontVelocity;
  float backVelocity;
};

struct RobotStatus {
  float x;
  float y;
  float heading;

  float vx;
  float vy;
  float vth;
};

#endif
