#include "OmniBot.h"
#include <stdio.h>
#include <unistd.h>

int main() {
    printf("hi\n");
    OmniBot bot("10.49.77.2");
    printf("hello\n");
    while ( true ) {
        RobotPose pose = bot.getPose();
        printf("Robot at %2.2f %2.2f\n", pose.x, pose.y);
        usleep(10000);
    }
    return 0;
}