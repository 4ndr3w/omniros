#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdio.h>
#define BUFLEN 8196

int main() {
    int sock;
    if ( (sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
      perror("socket() failed");
    }

    int tru = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &tru, sizeof(int));

    sockaddr_in bindAddr;
    bindAddr.sin_family = AF_INET;
    bindAddr.sin_addr.s_addr = INADDR_ANY;
    bindAddr.sin_port = htons(6666);

    if ( bind(sock, (sockaddr*)&bindAddr, sizeof(sockaddr_in)) < 0 )
        perror("bind() failed"); 
    while ( 1 ) {
        char buf[BUFLEN] = {0};
        if ( recv(sock, &buf, BUFLEN, 0) < 0 ) {
            perror("recv()");
            break;
        }
        printf("%s", buf);
    }

    close(sock);
}