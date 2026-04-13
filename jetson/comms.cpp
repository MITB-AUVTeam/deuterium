#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

#define RASPI_SOF0 0xAA
#define RASPI_SOF1 0x55

int fd = 0;

uint16_t calccrc(uint8_t* data) {
  uint16_t crc = 0xFFFF;   // initial value

  for (uint16_t i = 0; i < 13; i++) {
    crc ^= (uint16_t)data[i] << 8;

    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }

  return crc;
}


void uartInit() {
  const char* port = "/dev/ttyTHS1";

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

  tty.c_cc[VMIN] = 1;   //change later
  tty.c_cc[VTIME] = 1;  //timeout, change later

  tcsetattr(fd, TCSANOW, &tty);
}

void startMCU() {
  uint8_t msg[2] = { RASPI_SOF0, RASPI_SOF1 };
  write(fd, msg, 2);

  uint8_t b;
  uint8_t prev = 0;

  for (;;) {
    if (read(fd, &b, 1) == 1) {
      if (prev == 0xAA && b == 0x55)
        return;
      prev = b;
    }
  }
}

void sendNav(float dx, float dyaw, float ref_z, uint8_t id) {
  uint8_t buffer[17];

  buffer[0] = RASPI_SOF0;
  buffer[1] = RASPI_SOF1;

  buffer[2] = id;
  std::memcpy(&buffer[3], &dx, 4);
  std::memcpy(&buffer[7], &dyaw, 4);
  std::memcpy(&buffer[11], &ref_z, 4);

  uint16_t crc = calccrc(&buffer[2]);

  buffer[15] = crc & 0xFF;
  buffer[16] = (crc >> 8) & 0xFF;

  write(fd, buffer, 17);
}

void readDepth() {
  uint8_t b, prev = 0;

  for (;;) {
    if (read(fd, &b, 1) == 1) {
      if (prev == 0xAA && b == 0x55)
        break;
      prev = b;
    }
  }

  uint8_t data[4];
  if (read(fd, data, 4) != 4) return;

  uint32_t raw = data[0] | (data[1] << 8) |
    (data[2] << 16) | (data[3] << 24);

  float z;
  std::memcpy(&z, &raw, sizeof(z));
  std::cout << z << "\n";
}

int main() {

  uartInit();

  startMCU();
  std::cout << "success\n";

  for (;;) {
    sendNav(0, 0, 0.25, 1);
    readDepth();
  }

  close(fd);
  return 0;
}