#include <stdint.h>
#pragma once

typedef struct {
    float roll, pitch, z;
    float wx, wy, wz;
    float dx, dy, dz, dyaw;
} State;

typedef struct {
    uint16_t VB, VR, VL, HR, HL;
    int zoffset;
} Throttle;