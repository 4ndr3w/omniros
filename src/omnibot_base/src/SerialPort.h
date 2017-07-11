#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>

template <class SendMsg, class RecvMsg>
class SerialPort {
    int sock;
public:
    SerialPort(const char* port, speed_t baud) {
        printf("Opening %s\n", port);
        sock = open(port, O_RDWR);
        if ( sock == -1 ) {
            perror("open()");
            exit(1);
            return;
        }

        printf("open() done\n");

        struct termios settings;
        // Set baud rate
        tcgetattr(sock, &settings);
        cfsetospeed(&settings, baud);
        cfmakeraw(&settings);
        // Ensure blocking reads
        settings.c_cc[VMIN] = sizeof(RecvMsg);

        tcsetattr(sock, TCSANOW, &settings);
        tcflush(sock, TCIOFLUSH);
    }

    ~SerialPort() {
        close(sock);
    }

    RecvMsg getMessage() {
        RecvMsg data;
        if ( read(sock, &data, sizeof(RecvMsg)) < 1 ) {
            perror("read()");
            exit(1);
        }
        return data;
    }

    void sendMessage(SendMsg data) {
        write(sock, &data, sizeof(SendMsg));
    }
};


