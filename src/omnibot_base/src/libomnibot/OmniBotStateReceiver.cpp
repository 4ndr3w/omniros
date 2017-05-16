#include "OmniBotStateReceiver.h"
#include <netinet/in.h>
#include <sys/socket.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>



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

OmniBotStateReceiver::OmniBotStateReceiver()
    : running(true), stale(true), thread(&OmniBotStateReceiver::socketThread, this) {
        
}

OmniBotStateReceiver::~OmniBotStateReceiver() {
    close(sock);
    running = false;
    thread.join();
}

void OmniBotStateReceiver::socketThread() {
    if ( (sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
      perror("socket() failed");
    }

    sockaddr_in bindAddr;
    bindAddr.sin_family = AF_INET;
    bindAddr.sin_addr.s_addr = INADDR_ANY;
    bindAddr.sin_port = htons(1338);

    if ( bind(sock, (sockaddr*)&bindAddr, sizeof(sockaddr_in)) < 0 ) {
        perror("bind() failed");
    } 
    while ( running ) {
        sockaddr_in robotAddr;
        socklen_t len = sizeof(sockaddr_in);
        RobotState robot;

        recvfrom(sock, &robot, sizeof(RobotState), 0, (sockaddr*)&robotAddr, &len);

        // All the values we receive are big endian
        // byteSwap if we're running on a little endian machine
        robot.pose.x = byteSwap(robot.pose.x); 
        robot.pose.y = byteSwap(robot.pose.y); 
        robot.pose.theta = byteSwap(robot.pose.theta); 


        robot.pose.vx = byteSwap(robot.pose.vx); 
        robot.pose.vy = byteSwap(robot.pose.vy); 
        robot.pose.vth = byteSwap(robot.pose.vth); 

        robot.pose.forwardDistance = byteSwap(robot.pose.forwardDistance); 
/*
        robot.control.linearActual = byteSwap(robot.control.linearActual);
        robot.control.linearSetpoint = byteSwap(robot.control.linearSetpoint);

        robot.control.angularActual = byteSwap(robot.control.angularActual);
        robot.control.angularSetpoint = byteSwap(robot.control.angularSetpoint);
*/

        state = robot;
        stale = false;
    }
}

bool OmniBotStateReceiver::isStale() {
    return stale;
}

RobotState OmniBotStateReceiver::getState() {
    stale = true;
    return state;
}