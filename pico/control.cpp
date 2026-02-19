#include "control.hpp"
#include "structs.hpp"

extern State state;
extern Throttle throttle;

const float dt = 0.1;   // 100 ms control loop

//outer loop PID
float Kp_ang = 5.0;
float Ki_ang = 1.0;
float rollInt = 0;
float pitchInt = 0;

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
    if (value > 0)
        return constrain(value + 48, 0, 1000);
    else if (value <= 0)
        return constrain(-value + 1049, 1001, 2000);
    return 48;
}

void control::update() {

    rollInt += state.roll * dt;
    pitchInt += state.pitch * dt;

    // Limit integrator to prevent windup
    rollInt = constrain(rollInt, -0.02, 0.02);
    pitchInt = constrain(pitchInt, -0.02, 0.02);

    float wx_ref = Kp_ang * state.roll + Ki_ang * rollInt;
    float wy_ref = Kp_ang * state.pitch + Ki_ang * pitchInt;

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

    throttle.VL = clampDSHOT(u_smooth[0] * 150);
    throttle.VR = clampDSHOT(u_smooth[1] * 150);
    throttle.VB = clampDSHOT(u_smooth[2] * 150);
    // printf("%d      %d      %d\n", throttle.VL, throttle.VR, throttle.VB);
}
