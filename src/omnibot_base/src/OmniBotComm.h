#ifndef OMNIBOT_COMM
#define OMNIBOT_COMM
#include <stdint.h>

struct RobotCommand {
  float leftVelocity;
  float rightVelocity;
  float frontVelocity;
  float backVelocity;

  uint32_t checksum;
};

struct RobotStatus {
  float x;
  float y;
  float heading;

  float vx;
  float vy;
  float vth;

  uint32_t checksum;
};

#endif
