#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include "pico/stdlib.h"
#include "config.hpp"
#include "structs.hpp"
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "control.hpp"
#pragma once

class raspi {
public:
    static void init();

    static void blockforMPU();

    static bool update();

    static void sendpres();
};