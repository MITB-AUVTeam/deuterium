#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/time.h"

#include <stdint.h>
#include <inttypes.h>

#include "structs.hpp"
#include "imu.hpp"
#include "stb.hpp"
#include "esc.hpp"
#include "raspi.hpp"

volatile bool control_flag = false;
struct repeating_timer control_timer;

bool control_timer_cb(struct repeating_timer* t)
{
    control_flag = true;
    return true;
}

State state;
Throttle throttle = { 0, 0, 0, 0, 0, 0 };

int main(void) {

    // gpio_init(15);
    // gpio_set_dir(15, GPIO_OUT);
    // gpio_put(15, 0);

    stdio_init_all();

    sleep_ms(5000);
    printf("program initiating\n");

    imu::init();
    esc::pio_init();
    esc::arm();
    esc::mode3d();
    raspi::init();

    printf("program initialised\n");

    add_repeating_timer_ms(
        -AUV_STB_LOOP_MS,
        control_timer_cb,
        NULL,
        &control_timer
    );

    while (1) {

        raspi::update();

        if (control_flag) {
            control_flag = false;
            imu::update();
            stb::update();
        }

        printf("%d      %d      %d\n", throttle.VB, throttle.VR, throttle.VL);

        esc::thrust();
    }

}