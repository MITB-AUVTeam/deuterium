#include "esc.hpp"
#include "dshot.pio.h"

extern Throttle throttle;

PIO pio[5];
uint sm[5];
uint offset[5];
static const uint thruster[5] = { PIO_VB, PIO_VR, PIO_VL, PIO_HR, PIO_HL };

void esc::pio_init() {
    for (int i = 0;i < 5;i++) {
        bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&dshot_program, &pio[i],
            &sm[i], &offset[i], thruster[i], 1, true);
        hard_assert(success);
        dshot_program_init(pio[i], sm[i], offset[i], thruster[i]);
    }
}

void esc::arm() {
    for (int i = 0;i < 500;i++) {
        for (int j = 0;j < 5;j++) {
            pio_sm_put_blocking(pio[j], sm[j], 0x00000000 << 16); //arming
        }
        sleep_us(700);
    }
}

void esc::mode3d() {
    for (int i = 0;i < 10;i++) {
        for (int j = 0;j < 5;j++) {
            pio_sm_put_blocking(pio[j], sm[j], (uint32_t)0x0145 << 16); //3dmode
        }
        sleep_us(700);
    }
}

void esc::thrust() {
    uint16_t throttleesc[5] = { throttle.VB,throttle.VR,throttle.VL,throttle.HR,throttle.HL };
    for (int j = 0;j < 5;j++) {
        throttleesc[j] &= 0x7FF;
        uint16_t packet = (throttleesc[j] << 1) | 0;
        uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;         //calulating 4bit CRC
        uint16_t escframe = (packet << 4) | crc;        //final 16bit frame that needs to be sent
        pio_sm_put_blocking(pio[j], sm[j], (uint32_t)escframe << 16);
    }
    sleep_us(700);
}