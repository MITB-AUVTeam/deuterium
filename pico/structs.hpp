#include <stdint.h>
#pragma once

struct State {
    float roll, pitch, yaw, z;
    float ref_roll, ref_z;
    float wx, wy, wz;
    float vx, dy, dyaw;

    State() {
        roll = pitch = z = 0;
        ref_roll = ref_z = 0;
        wx = wy = wz = 0;
        vx = dy = dyaw = 0;
    }
};

struct Throttle {
    uint16_t VB, VR, VL, HR, HL;
    int zoffset;

    Throttle() {
        VB = VR = VL = HR = HL = 0;
        zoffset = 0;
    }
};