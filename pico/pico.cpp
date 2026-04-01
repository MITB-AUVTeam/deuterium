#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/time.h"

#include <stdint.h>
#include <inttypes.h>

#include "structs.hpp"
#include "imu.hpp"
#include "control.hpp"
#include "esc.hpp"
#include "raspi.hpp"
#include "pressure.hpp"

volatile bool stb_flag = false;
bool nav_data_flag = false;
bool nav_time_out = true;       //starts is safe consdition

absolute_time_t last_nav_data_time = get_absolute_time();
absolute_time_t new_nav_data_time = get_absolute_time();

absolute_time_t stopper = get_absolute_time();

struct repeating_timer control_timer;

bool control_timer_cb(struct repeating_timer* t)
{
    stb_flag = true;
    return true;
}

State state;
Throttle throttle;

int main(void) {

    // gpio_init(15);
    // gpio_set_dir(15, GPIO_OUT);
    // gpio_put(15, 0);

    stdio_init_all();

    sleep_ms(3000);
    printf("program initiating\n");

    // imu::init();

    // presens::init();

    esc::pio_init();

    esc::arm();
    esc::mode3d();
    raspi::init();

    // sleep_ms(240000);

    printf("program initialised\n");

    add_repeating_timer_ms(
        -STB_LOOP_MS,
        control_timer_cb,
        NULL,
        &control_timer
    );

    for (;;) {

        if (stb_flag) {
            stb_flag = false;
            // imu::update();
            // presens::read();
            // control::stbUpdate();
        }

        nav_data_flag = raspi::update();

        if (nav_data_flag) {
            new_nav_data_time = get_absolute_time();
            float nav_dt = absolute_time_diff_us(last_nav_data_time, new_nav_data_time) / 1000000.0f;
            last_nav_data_time = new_nav_data_time;
            nav_time_out = false;
            // control::navUpdate(nav_dt);
            printf("%f      %f      %f\n", state.dx, state.dyaw, state.ref_z);
        }
        if (!nav_time_out && absolute_time_diff_us(last_nav_data_time, get_absolute_time()) > NAV_TIME_OUT_US) {
            // control::navStop();
            nav_time_out = true;
            printf("timeoout");
        }

        // printf("%d      %d      %d\n", throttle.VB, throttle.VR, throttle.VL);


        esc::thrust();
    }

}