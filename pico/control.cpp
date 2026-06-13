#include "control.hpp"

#include "structs.hpp"

#include "thrustLUT.hpp"

extern State state;

extern Throttle throttle;


struct PID {
    float kp, ki, kd;
    float prev;
    float integral;
};
PID pid_roll = { 10, 0, 50, 0, 0 };
PID pid_pitch = { 0, 0, 0, 0, 0 };
PID pid_z = { 60, 10, 10, 0, 0 };
PID pid_x = { 1, 0, 0, 0, 0 };
PID pid_yaw = { 7.5, 0, 1, 0, 0 };

// LQR
float K_tau[2][2] = {
    {0.2969, 0},
    {0, 0.2876}
};

// constants
const float STB_LOOP_DT = STB_LOOP_MS / 1000.0f;

const float Fz_eq = 12;
const float F_MIN = -23.3f;
const float F_MAX = 29.8f;
const float tau_scale = 1.0f;

//XtoF MATRIX
const float XtoF[3][3] = {
    {0, -2.0000, 0.1},
    {-2.2222, 1.0000, 0.2},
    {2.2222, 1.0000, 0.2}
};


float constrain(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

float computePID(PID& p, float error, float dt) {

    p.integral += error * dt;
    p.integral = constrain(p.integral, -1, 1);

    float derivative = (error - p.prev) / dt;

    float output = p.kp * error + p.ki * p.integral + p.kd * derivative;
    p.prev = error;

    return output;
}

void control::stbUpdate() {

    //outer loop PID
    float wx_ref = computePID(pid_roll, -state.roll, STB_LOOP_DT);
    float wy_ref = computePID(pid_pitch, -state.pitch, STB_LOOP_DT);

    //inner loop lqr
    float omega_err_x = state.wx - wx_ref;
    float omega_err_y = state.wy - wy_ref;
    float tau_roll = -tau_scale * (K_tau[0][0] * omega_err_x + K_tau[0][1] * omega_err_y);
    float tau_pitch = -tau_scale * (K_tau[1][0] * omega_err_x + K_tau[1][1] * omega_err_y);

    //z control
    float z_error = state.ref_z - state.z;
    float Fz_pid = computePID(pid_z, z_error, STB_LOOP_DT);
    float Fz = Fz_eq + Fz_pid;
    // float Fz = Fz_eq;

    //x to f mixing
    // float VB = XtoF[0][0] * tau_roll + XtoF[0][1] * tau_pitch + XtoF[0][2] * Fz;
    // float VR = XtoF[1][0] * tau_roll + XtoF[1][1] * tau_pitch + XtoF[1][2] * Fz;
    // float VL = XtoF[2][0] * tau_roll + XtoF[2][1] * tau_pitch + XtoF[2][2] * Fz;
    float VB = XtoF[0][2] * Fz;
    float VR = XtoF[1][2] * Fz;
    float VL = XtoF[2][2] * Fz;

    //saturation
    VB = constrain(VB, F_MIN, F_MAX);
    VR = constrain(VR, F_MIN, F_MAX);
    VL = constrain(VL, F_MIN, F_MAX);

    throttle.VL = thrustToDshot(-VL);
    throttle.VR = thrustToDshot(VR);
    throttle.VB = thrustToDshot(VB);
}

void control::navUpdate(float nav_dt) {

    float x = computePID(pid_x, -state.dx, nav_dt);
    float yaw = computePID(pid_yaw, -state.dyaw, nav_dt);
    float HL = x - yaw;
    float HR = x + yaw;

    HL = constrain(HL, F_MIN, F_MAX);
    HR = constrain(HR, F_MIN, F_MAX);

    throttle.HL = thrustToDshot(HL);
    throttle.HR = thrustToDshot(HR);

}

void control::navStop() {
    throttle.HL = 48;
    throttle.HR = 48;
}