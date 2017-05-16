#include "OmniBotCommandSender.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <stdio.h>
#include <errno.h>
#include <cstdlib>
#include <string.h>
#include <assert.h>

OmniBotCommandSender::OmniBotCommandSender(const char* address) {
    if ( (sock = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
      perror("socket() failed");
      exit(-1);
    }

    memset(&addr, 0, sizeof(sockaddr_in));
  
    // Setup the robot's addr struct
    addr.sin_addr.s_addr = inet_addr(address);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(1337);
    
    assert(connect(sock, (sockaddr*)&addr, sizeof(sockaddr)) != -1);
}

RobotResponse OmniBotCommandSender::pushCommand(RobotMessage command) {
    RobotResponse response;
    response.result = false;
    send(sock, &command, sizeof(RobotMessage), 0);
    recv(sock, &response, sizeof(RobotResponse), 0);
    return response;
}
