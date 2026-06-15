#include <stdio.h>
#include "pico/stdlib.h"
#include <stdint.h>
#include <inttypes.h>
#include "pico/time.h"
#include "hardware/pio.h"



#include "config.hpp"
#include "structs.hpp"
#pragma once

class esc {
public:
    static void pio_init();
    static void arm();
    static void mode3d();
    static void thrust();
};