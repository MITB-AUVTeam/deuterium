#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "pico/multicore.h"

#include <stdint.h>
#include <inttypes.h>

#include "config.hpp"
#include "structs.hpp"
#include "imu.hpp"
#include "control.hpp"
#include "esc.hpp"
#include "raspi.hpp"
#include "pressure.hpp"

bool nav_data_flag = false;
bool nav_time_out = true;       //starts is safe consdition

absolute_time_t last_nav_data_time = get_absolute_time();
absolute_time_t new_nav_data_time = get_absolute_time();

absolute_time_t stopper = get_absolute_time();

struct repeating_timer control_timer;
volatile bool esc_flag = false;
static uint32_t timer_count = 0;

bool control_timer_cb(struct repeating_timer* t) {

    esc_flag = true;

    if (++timer_count >= STB_LOOP_MS) {
        timer_count = 0;
    }
    return true;
}

State state;
Throttle throttle;

void core1_entry() {
    for (;;) {
#if !DEBUG_MODE
        nav_data_flag = raspi::update();
        raspi::sendpres();

        if (nav_data_flag) {
            new_nav_data_time = get_absolute_time();
            float nav_dt = absolute_time_diff_us(last_nav_data_time, new_nav_data_time) / 1000000.0f;
            last_nav_data_time = new_nav_data_time;
            nav_time_out = false;
            control::navUpdate(0.02);          //change the dt here, its supposed to be nav_dt but not working maybe because too fast or, nav_ft becomes 0 becuase number too small ig
        }
        if (!nav_time_out && absolute_time_diff_us(last_nav_data_time, get_absolute_time()) > NAV_TIME_OUT_US) {
            control::navStop();
            nav_time_out = true;
        }
#endif
#if DEBUG_MODE
        printf("%f\t%f\t\t", state.roll, state.pitch);
        printf("%f\t\t", state.z);
        printf("%d\t%d\t%d\t%d\t%d\n", throttle.VB, throttle.VR, throttle.VL, throttle.HL, throttle.HR);
#endif
        sleep_ms(20);
    }
}
int main(void) {

    stdio_init_all();

#if DEBUG_MODE
    sleep_ms(1000);
    gpio_init(4);
    gpio_set_dir(4, true);
    bool dummy = true;
    while (!stdio_usb_connected()) {
        sleep_ms(100);
        gpio_put(4, dummy);
        dummy = !dummy;
    }
    sleep_ms(1000);
    printf("program initiating\n");
#endif

    raspi::init();
    // raspi::blockforMPU();

    gpio_init(DEBUG_PIN);
    gpio_set_dir(DEBUG_PIN, GPIO_OUT);
    gpio_put(DEBUG_PIN, 0);

    imu::init();
    presens::init();

    multicore_launch_core1(core1_entry);

    esc::pio_init();
    esc::arm();
    esc::mode3d();

#if DEBUG_MODE
    printf("program initialised\n");
#endif

    add_repeating_timer_ms(-1, control_timer_cb, NULL, &control_timer);

    for (;;) {

        if (esc_flag) {
            esc_flag = false;
            uint32_t control_count = timer_count;
            // throttle.VB = 48;
            // throttle.VR = 48;
            // throttle.VL = 48;
            // throttle.HR = 48;
            // throttle.HL = 300;

            esc::thrust();

            switch (control_count) {
            case 1:
                imu::ask_euler();
                break;
            case 3:
                imu::read_euler();
                break;
            case 4:
                imu::ask_gyro();
                break;
            case 5:
                imu::read_gyro();
                break;
            case 6:
                presens::ask_D1_5();
                break;
            case 11:
                presens::read_D1_0();
                presens::ask_D2_5();
                break;
            case 16:
                presens::read_D2_0();
                presens::calc_depth_0();
                control::stbUpdate();
                break;

            default:
                break;
            }
        }
        if(state.ref_z < 0) {
            throttle.VB = 48;
            throttle.VR = 48;
            throttle.VL = 48;
            throttle.HR = 48;
            throttle.HL = 48;
            esc::thrust();
            // return;
        }

        // imu::ask_euler();
        // imu::read_euler();
        // imu::ask_gyro();
        // imu::read_gyro();

        // presens::ask_D1_5();
        // sleep_ms(5);
        // presens::read_D1_0();
        // presens::ask_D2_5();
        // sleep_ms(5);
        // presens::read_D2_0();
        // presens::calc_depth_0();
        // control::stbUpdate();

    }
}