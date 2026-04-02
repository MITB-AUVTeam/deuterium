#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <cstdint>

#define RASPI_SOF0 0xAA
#define RASPI_SOF1 0x55

int fd=0;

void uartInit(){
        const char* port = "/dev/ttyAMA0";

    fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening port\n";
        return;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error getting attributes\n";
        return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag = 0;

    tcsetattr(fd, TCSANOW, &tty);
}

void startMCU() {
    uint8_t msg[2];
    msg[0] = RASPI_SOF0;
    msg[1] = RASPI_SOF1;

    write(fd, msg, sizeof(msg));

    for(;;){
        uint8_t data[2]={0,0};
        ssize_t n = read(fd, &data, 2);
        if (data[0] == 0xAA && data[1] == 0x55)
            return;
    }
}

void readDepth(){
    uint8_t data[6];
    if (data[0] == 0xAA && data[1] == 0x55) {
        uint32_t raw = data[2] |
                   (data[3] << 8) |
                   (data[4] << 16) |
                   (data[5] << 24);

        float z;
        std::memcpy(&z, &raw, sizeof(z));
        std::cout <<z<<"\n";
    }
}

int main() {

    uartInit();

    startMCU();
    std::cout<<"success\n";

    for(;;){
        readDepth();
    }

    close(fd);
    return 0; 
}