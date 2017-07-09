#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>


template <class SendMsg, class RecvMsg>
class SerialPort {
    int sock;
public:
    SerialPort(const char* port, speed_t baud) {
        printf("Opening %s\n", port);
        sock = open(port, O_RDWR);
        if ( sock == -1 ) {
            perror("open()");
            return;
        }

        printf("open() done\n");

        struct termios settings;
        // Set baud rate
        tcgetattr(sock, &settings);
        cfsetospeed(&settings, baud);
        tcsetattr(sock, TCSANOW, &settings);
        tcflush(sock, TCOFLUSH);

        printf("construct\n");
    }

    ~SerialPort() {
        close(sock);
    }

    RecvMsg getMessage() {
        RecvMsg data;
        read(sock, &data, sizeof(RecvMsg));
        return data;
    }

    void sendMessage(SendMsg data) {
        write(sock, &data, sizeof(SendMsg));
    }
};