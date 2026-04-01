#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "structs.hpp"
#include "config.hpp"
#pragma once


class presens {
public:
    static void init();

    static void read();
};