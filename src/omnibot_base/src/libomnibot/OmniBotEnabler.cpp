#include <stdio.h>
#include <unistd.h>
#include <cstdlib>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <zlib.h>
#include <string.h>

#include "frcds.h"

uint16_t uint16_t_swap(uint16_t val) {
	return (val>>8) | (val<<8);
}

uint32_t uint32_t_swap(uint32_t num) {
	return ((num>>24)&0xff) | // move byte 3 to byte 0
		            ((num<<8)&0xff0000) | // move byte 1 to byte 2
		            ((num>>8)&0xff00) | // move byte 2 to byte 1
		            ((num<<24)&0xff000000); // byte 0 to byte 3
}

int main(int argc, char** argv){
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in addr;
  memset(&addr, 0, sizeof(sockaddr_in));

  // Connect to robot controller
  addr.sin_addr.s_addr = inet_addr("10.49.77.2");
  addr.sin_family = AF_INET;
  addr.sin_port = htons(1110);

  DrivePacket *pkt = (DrivePacket*)malloc(1024);
  memset(pkt, 0, 1024);
  pkt->control = 0x70; // Auto Enabled
  pkt->teamID = uint16_t_swap(4977);
  pkt->dsDigitalIn = 0xff;
  pkt->dsAlliance = 0x52;
  pkt->dsPosition = 0x31;

  
  int idx = 0;
  while ( 1 ) {
    pkt->packetIndex = uint16_t_swap(idx++);

    unsigned long *crc = (unsigned long*)(((char*)pkt)+(1024-4));
    *crc = crc32(0L, Z_NULL, 0);
    *crc = uint32_t_swap(crc32(0, (Bytef*)pkt, 1024));

    if ( sendto(sock, pkt, 1024, 0, (sockaddr*)&addr, sizeof(sockaddr_in)) < 0 )
	    perror("Failed to send restart command to robot controller\n");
    usleep(20000); // 50hz
  }
  free(pkt);
  close(sock);
}