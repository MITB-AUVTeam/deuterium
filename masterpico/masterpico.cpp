#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/time.h"

#include <stdint.h>
#include <inttypes.h>

#include "dshot.pio.h"

#include "structs.hpp"
#include "imu.hpp"
#include "control.hpp"

volatile bool control_flag = false;
struct repeating_timer control_timer;

bool control_timer_cb(struct repeating_timer* t)
{
    control_flag = true;
    return true;
}

State state;
Throttle throttle = { 0, 0, 0, 0, 0, 0 };

PIO pio[5];
uint sm[5];
uint offset[5];
static const uint thruster[5] = { 5, 6, 7, 8, 9 };

void allthrusters_init() {
    for (int i = 0;i < 5;i++) {
        bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&dshot_program, &pio[i],
            &sm[i], &offset[i], thruster[i], 1, true);
        hard_assert(success);
        dshot_program_init(pio[i], sm[i], offset[i], thruster[i]);
    }
}

void arm_thrusters() {
    for (int i = 0;i < 500;i++) {
        for (int j = 0;j < 5;j++) {
            pio_sm_put_blocking(pio[j], sm[j], 0x00000000 << 16); //arming
            sleep_us(700);
        }
    }
    for (int i = 0;i < 10;i++) {
        for (int j = 0;j < 5;j++) {
            pio_sm_put_blocking(pio[j], sm[j], (uint32_t)0x0145 << 16); //3dmode
            sleep_us(700);
        }
    }
}

int main(void) {

    // gpio_init(15);
    // gpio_set_dir(15, GPIO_OUT);
    // gpio_put(15, 0);

    stdio_init_all();

    sleep_ms(5000);
    printf("program initiating\n");

    imu::init();
    allthrusters_init();
    arm_thrusters();

    printf("program initialised\n");

    add_repeating_timer_ms(
        -100,                //100ms control loop
        control_timer_cb,
        NULL,
        &control_timer
    );

    while (1) {

        if (control_flag) {
            control_flag = false;
            imu::update();
            control::update();
        }

        // printf("%d      %d      %d\n", throttle.VB, throttle.VR, throttle.VL);

        uint16_t throttleesc[5] = { 48,throttle.VR,throttle.VL,throttle.HR,throttle.HL };
        for (int j = 0;j < 5;j++) {
            throttleesc[j] &= 0x7FF;
            printf("%d      ", throttleesc[j]);
            uint16_t packet = (throttleesc[j] << 1) | 0;
            uint16_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;         //calulating 4bit CRC
            uint16_t escframe = (packet << 4) | crc;        //final 16bit frame that needs to be sent
            pio_sm_put_blocking(pio[j], sm[j], (uint32_t)escframe << 16);
        }
        printf("\n");

        sleep_us(700);
    }

}