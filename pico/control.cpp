#include "control.hpp"
#include "structs.hpp"

extern State state;
extern Throttle throttle;

struct PID {
    float kp;
    float ki;
    float kd;
    float windup;
    float prev;
    float integral;
};


//navigation
PID pidx = { 1.2, 0.01, 0.25, 100, 0, 0 };
PID pidy = { 1.2, 0.01, 0.25, 100, 0, 0 };
PID pidz = { 1.5, 0.02, 0.3, 100, 0, 0 };
PID pidyaw = { 2.5, 0.01, 0.3, 100, 0, 0 };


//stabalization
PID pidroll = { 5.0, 1.0, 0, 0.02, 0, 0 };
PID pidpitch = { 5.0, 1.0, 0, 0.02, 0, 0 };
const float stb_dt = STB_LOOP_MS / 1000.0f;
//inner loop LQR
float K_lqr[3][2] = {
  { 1.5, -1},
  { -1.5, -1 },
  { 0.0, 2.0 }
};
const float U_MAX = 1.0;
float u_smooth[3] = { 0, 0, 0 };
const float beta = 0.2; // LQR output smoothing factor


static inline float constrain(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

int clampDSHOT(int value) {
    if (value >= 0)
        return constrain(value + 48, 0, 1000);
    else if (value < 0)
        return constrain(-value + 1049, 1001, 2000);
    return 48;
}

float computePID(PID& p, float error, float dt)
{
    if (dt <= 0) return 0;

    p.integral += error * dt;
    p.integral = constrain(p.integral, -p.windup, p.windup);

    float derivative = (error - p.prev) / dt;

    float output = p.kp * error +
        p.ki * p.integral +
        p.kd * derivative;

    p.prev = error;

    return output;
}


void control::stbUpdate() {

    float wx_ref = computePID(pidroll, state.roll, stb_dt);
    float wy_ref = computePID(pidpitch, state.pitch, stb_dt);

    //inner loop LQR
    float omega_err[2] = { state.wx - wx_ref, state.wy - wy_ref };

    // Deadband for gyro errors
    for (int i = 0; i < 2; i++) {
        if (std::fabs(omega_err[i]) < 0.01) omega_err[i] = 0;
    }

    float u[3];
    u[0] = -(K_lqr[0][0] * omega_err[0] + K_lqr[0][1] * omega_err[1]);
    u[1] = -(K_lqr[1][0] * omega_err[0] + K_lqr[1][1] * omega_err[1]);
    u[2] = -(K_lqr[2][0] * omega_err[0] + K_lqr[2][1] * omega_err[1]);

    // Normalize LQR outputs
    float umax = std::max(std::max(std::fabs(u[0]), std::fabs(u[1])), std::fabs(u[2]));
    if (umax > U_MAX) {
        u[0] *= U_MAX / umax;
        u[1] *= U_MAX / umax;
        u[2] *= U_MAX / umax;
    }

    //lqr smooth output
    for (int i = 0; i < 3; i++) {
        u_smooth[i] = beta * u[i] + (1 - beta) * u_smooth[i];
    }

    throttle.VL = clampDSHOT((u_smooth[0] * 150) + state.z);   //z is being adjusted in nav loop
    throttle.VR = clampDSHOT((u_smooth[1] * 150) + state.z);
    throttle.VB = clampDSHOT((u_smooth[2] * 150) + state.z);
    // printf("%d      %d      %d\n", throttle.VL, throttle.VR, throttle.VB);
}

void control::navUpdate(float nav_dt)
{

    // if (abs(state.dx) < 0.05) state.dx = 0;
    // if (abs(state.dy) < 0.05) state.dy = 0;
    // if (abs(state.dz) < 0.05) state.dz = 0;                               // deadbands and angle warp can be applied in mpu ???

    float ux = computePID(pidx, state.dx, nav_dt);
    float uy = computePID(pidy, state.dy, nav_dt);
    float uz = computePID(pidz, state.dz, nav_dt);
    float uyaw = computePID(pidyaw, state.dyaw, nav_dt);

    ux = constrain(ux, -300, 300);
    uy = constrain(uy, -300, 300);
    uz = constrain(uz, -300, 300);
    uyaw = constrain(uyaw, -200, 200);

    throttle.HL = clampDSHOT((ux - uyaw) * 100);
    throttle.HR = clampDSHOT((ux + uyaw) * 100);
    state.z = uz;
    state.ref_roll = uy;

}

void control::navStop() {
    return;
}
