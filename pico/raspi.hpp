#include "hardware/uart.h"
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include "pico/stdlib.h"
#include "config.hpp"
#include "structs.hpp"
#pragma once

class raspi {
public:
    static void init();

    static void blockforMPU();

    static bool update();

    static void sendpres();
};