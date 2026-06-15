#include "raspi.hpp"

extern State state;

int recstate = 0;
int recbuffindex = 0;
uint8_t recbuff[13];

void raspi::init() {
    stdio_usb_init();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
}

bool raspi::update() {

    while (true) {

        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT)
            break;
        uint8_t byte = (uint8_t)c;

        switch (recstate) {

        case 0:
            if (byte == RASPI_SOF0)
                recstate = 1;
            break;

        case 1:
            if (byte == RASPI_SOF1) {
                recstate = 2;
                recbuffindex = 0;
            }
            else {
                recstate = 0;
            }
            break;

        case 2:
            recbuff[recbuffindex++] = byte;
            if (recbuffindex >= 13) {

                memcpy(&state.dyaw, &recbuff[0], 4);
                memcpy(&state.dx, &recbuff[4], 4);
                memcpy(&state.ref_z, &recbuff[8], 4);
                if (recbuff[12] == 1)
                    control::navStop();

                recbuffindex = 0;
                recstate = 0;

                return true;
            }
            break;

        default:
            recbuffindex = 0;
            recstate = 0;
        }
    }
    return false;
}

void raspi::sendpres() {
    uint8_t packet[6];
    packet[0] = RASPI_SOF0;
    packet[1] = RASPI_SOF1;
    memcpy(&packet[2], &state.z, 4);
    fwrite(packet, 1, sizeof(packet), stdout);
    fflush(stdout);
}