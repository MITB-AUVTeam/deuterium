#include "control.hpp"
#include "structs.hpp"
#include "thrustLUT.hpp"

extern State state;
extern Throttle throttle;

// ================= PID =================
struct PID {
    float kp, ki, kd;
    float prev;
    float integral;
};

PID pid_phi = { 2.87, 0.33, 4.89, 0, 0 };
PID pid_theta = { 2.76, 0.83, 1.12, 0, 0 };
PID pid_z = { 1.5, 0.02, 0.3, 0, 0 };

// LQR
float K_tau[2][2] = {
    {0.2969, 0},
    {0, 0.2878}
};

// ================= CONSTANTS =================
const float dt = STB_LOOP_MS / 1000.0f;

const float Kp_ang = 5.0;
const float Ki_ang = 1.0;

const float beta = 0.2;
float u_smooth[3] = { 0,0,0 };

const float U_MAX = 1.0;

const float m = 20.0;
const float g = 9.8;
const float Fb = 196.0;

const float Fz_eq = m * g - Fb;

const float F_MIN = -23.3f;
const float F_MAX = 29.8f;

// ================= XtoF MATRIX =================
const float XtoF[3][3] = {
    { 0.00,  2.00,  10},
    {-2.22,   -1.00,   5},
    { 2.22,   -1.00,   5}
};

// ================= HELPERS =================
float constrain(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

float computePID(PID& p, float error) {
    p.integral += error * dt;
    p.integral = constrain(p.integral, -100, 100);

    float derivative = (error - p.prev) / dt;

    float output = p.kp * error +
        p.ki * p.integral +
        p.kd * derivative;

    p.prev = error;
    return output;
}

void applyDeadband(float& val) {
    if (fabs(val) < 0.01) val = 0;
}

void control::stbUpdate() {

    // ---------- OUTER LOOP (ANGLE → ω_ref) ----------
    static float phiInt = 0;
    static float thetaInt = 0;

    phiInt += state.roll * dt;
    thetaInt += state.pitch * dt;

    phiInt = constrain(phiInt, -0.02, 0.02);
    thetaInt = constrain(thetaInt, -0.02, 0.02);

    float wx_ref = Kp_ang * state.roll + Ki_ang * phiInt;
    float wy_ref = Kp_ang * state.pitch + Ki_ang * thetaInt;

    // ---------- INNER LOOP (LQR) ----------
    float omega_err_x = state.wx - wx_ref;
    float omega_err_y = state.wy - wy_ref;

    // applyDeadband(omega_err_x);
    // applyDeadband(omega_err_y);

    float tau_phi = -(K_tau[0][0] * omega_err_x + K_tau[0][1] * omega_err_y);
    float tau_theta = -(K_tau[1][0] * omega_err_x + K_tau[1][1] * omega_err_y);

    // ---------- SMOOTHING ----------
    static float tau_phi_s = 0;
    static float tau_theta_s = 0;

    tau_phi_s = beta * tau_phi + (1 - beta) * tau_phi_s;
    tau_theta_s = beta * tau_theta + (1 - beta) * tau_theta_s;

    tau_phi = tau_phi_s;
    tau_theta = tau_theta_s;

    // ---------- Z CONTROL ----------

    static float z_ref = 0.25;

    float z_error = z_ref - state.z;
    // float z_error = 0;


    float Fz_pid = computePID(pid_z, z_error);
    float Fz = Fz_eq + Fz_pid;

    // printf("%f      %f      %f      ", tau_phi, tau_theta, Fz);

    // ---------- XtoF MIXING ----------

    float F1 = XtoF[0][0] * tau_phi + XtoF[0][1] * tau_theta + XtoF[0][2] * Fz;
    float F2 = XtoF[1][0] * tau_phi + XtoF[1][1] * tau_theta + XtoF[1][2] * Fz;
    float F3 = XtoF[2][0] * tau_phi + XtoF[2][1] * tau_theta + XtoF[2][2] * Fz;

    // printf("%f      %f      %f        ", F1, F2, F3);

    // ---------- SATURATION ----------
    F1 = constrain(F1, F_MIN, F_MAX);
    F2 = constrain(F2, F_MIN, F_MAX);
    F3 = constrain(F3, F_MIN, F_MAX);

    // printf("%f      %f      %f\n", F1, F2, F3);

    // ---------- LUT → DSHOT ----------
    throttle.VL = thrustToDshot(F1);
    throttle.VR = thrustToDshot(F2);
    throttle.VB = thrustToDshot(F3);
}