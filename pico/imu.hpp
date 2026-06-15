#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdint.h>
#include <cmath>
#include "structs.hpp"
#include "config.hpp"
#pragma once


class imu {
public:
    static void init();
    static void ask_euler();
    static void read_euler();
    static void ask_gyro();
    static void read_gyro();
};