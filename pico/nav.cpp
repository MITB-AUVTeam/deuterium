#include "nav.hpp"

extern State state;
extern Throttle throttle;

struct PID {
    float kp;
    float ki;
    float kd;
    float prev;
    float integral;
};

PID pidx = { 1.2, 0.01, 0.25, 0, 0 };
PID pidy = { 1.2, 0.01, 0.25, 0, 0 };
PID pidz = { 1.5, 0.02, 0.3, 0, 0 };
PID pidyaw = { 2.5, 0.01, 0.3, 0, 0 };

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
    p.integral = constrain(p.integral, -100, 100);

    float derivative = (error - p.prev) / dt;

    float output = p.kp * error +
        p.ki * p.integral +
        p.kd * derivative;

    p.prev = error;

    return output;
}

float wrapAngle(float angle)
{
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

float dt = 0.02;

void nav::update()
{

    if (fabs(state.dx) < 0.05) state.dx = 0;
    if (fabs(state.dy) < 0.05) state.dy = 0;
    if (fabs(state.dz) < 0.05) state.dz = 0;
    state.dyaw = wrapAngle(state.dyaw);

    float ux = computePID(pidx, state.dx, dt);
    float uy = computePID(pidy, state.dy, dt);
    float uz = computePID(pidz, state.dz, dt);
    float uyaw = computePID(pidyaw, state.dyaw, dt);

    ux = constrain(ux, -300, 300);
    uy = constrain(uy, -300, 300);
    uz = constrain(uz, -300, 300);
    uyaw = constrain(uyaw, -200, 200);

    float t1 = ux - uyaw;
    float t2 = ux + uyaw;
    float t3 = uz + uy;
    float t4 = uz - uy;
    float t5 = uz;

}
