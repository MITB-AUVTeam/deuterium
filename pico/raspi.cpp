#include "raspi.hpp"

extern State state;

int recstate = 0;
int recbuffindex = 0;
uint8_t recbuff[15];

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


void raspi::init() {
    uart_init(RASPI_UARTID, RASPI_BAUDRATE);
    gpio_set_function(RASPI_TX, GPIO_FUNC_UART);
    gpio_set_function(RASPI_RX, GPIO_FUNC_UART);
}

uint8_t id = 0;

bool raspi::update() {

    while (uart_is_readable(RASPI_UARTID)) {
        switch (recstate) {

        case 0:
            if (uart_getc(RASPI_UARTID) == RASPI_SOF0)
                recstate = 1;
            break;

        case 1:
            if (uart_getc(RASPI_UARTID) == RASPI_SOF1)
                recstate = 2;
            else
                recstate = 0;
            break;

        case 2:
            recbuff[recbuffindex] = uart_getc(RASPI_UARTID);
            recbuffindex++;
            if (recbuffindex >= 15) {
                uint16_t reccrc = recbuff[13] | (recbuff[14] << 8);
                uint16_t computedcrc = calccrc(recbuff);
                if (reccrc == computedcrc) {
                    memcpy(&id, &recbuff[0], 1);
                    memcpy(&state.dx, &recbuff[1], 4);
                    memcpy(&state.dyaw, &recbuff[5], 4);
                    memcpy(&state.ref_z, &recbuff[9], 4);
                }
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