#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "structs.hpp"
#include "config.hpp"
#pragma once


class presens {
public:
    static void init();

    static void ask_D1_5();

    static void read_D1_0();

    static void ask_D2_5();

    static void read_D2_0();

    static void calc_depth_0();
};