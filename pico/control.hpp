#include "pico/stdlib.h" 
#include <string.h>
#include <stdio.h>
#include <cmath>
#include <algorithm>
#pragma once
#include "structs.hpp"
#include "config.hpp"

class control {
public:
    static void init();
    static void stbUpdate();
    static void navUpdate(float nav_dt);
    static void navStop();
};