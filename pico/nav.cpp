#include "nav.hpp"

extern State state;

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

// -------- PID FUNCTION (DT BASED) --------
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


// -------- ANGLE WRAP --------
float wrapAngle(float angle)
{
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

unsigned long lastTime = 0;
const int loopInterval = 20; // ms (target ~50 Hz)


// -------- LOOP --------
void loop()
{
    unsigned long now = millis();

    if (now - lastTime >= loopInterval)
    {
        float dt = (now - lastTime) / 1000.0;
        lastTime = now;
        dt = constrain(dt, 0.001, 0.05);


        // -------- DEADBAND --------
        if (abs(state.dx) < 0.05) state.dx = 0;
        if (abs(state.dy) < 0.05) state.dy = 0;
        if (abs(state.dz) < 0.05) state.dz = 0;

        // -------- PID --------
        float ux = computePID(pidx, state.dx, dt);
        float uy = computePID(pidy, state.dy, dt);
        float uz = computePID(pidz, state.dz, dt);
        float uyaw = computePID(pidyaw, state.dyaw, dt);

        // -------- LIMITS --------
        ux = constrain(ux, -300, 300);
        uy = constrain(uy, -300, 300);
        uz = constrain(uz, -300, 300);
        uyaw = constrain(uyaw, -200, 200);

        // -------- THRUSTER MIXING --------
        float t1 = ux - uyaw;
        float t2 = ux + uyaw;

        float t3 = uz + uy;
        float t4 = uz - uy;
        float t5 = uz;
    }
}