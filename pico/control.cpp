#include "control.hpp"

#include "structs.hpp"

#include "thrustLUT.hpp"

extern State state;

extern Throttle throttle;

// ================= PID =================

struct PID
{

    float kp, ki, kd;

    float prev;

    float integral;

    float ref;
};

PID pid_roll = { 100, 50, 20, 0, 0, 0 };

PID pid_pitch = { -100, -50, 20, 0, 0, 0 };

PID pid_z = { 0, 0, 0, 0, 0, 0 };

PID pid_yaw = { 0, 0, 0, 0, 0, 0 };

PID* tune = &pid_pitch;

// LQR

float K_tau[2][2] = {

    {0.2969, 0},

    {0, 0.2876}

};

// ================= CONSTANTS =================

const float dt = STB_LOOP_MS / 1000.0f;

float u_smooth[3] = { 0, 0, 0 };

const float U_MAX = 1.0;

const float m = 20.0;

const float g = 9.8;

const float Fb = 250;

const float Fz_eq = m * g - Fb;

const float F_MIN = -23.3f;

const float F_MAX = 29.8f;

const float tau_scale = 1.0f;

// ================= XtoF MATRIX =================

const float XtoF[3][3] = {

    {0, -2.0000, 0.5600},

    {-2.2222, 1.0000, 0.2200},

    {2.2222, 1.0000, 0.2200}

};

// Globals

bool thrusterOff;

bool pidUpdate;

bool refUpdate;

#define BUF_SIZE 512

static char input_buf[BUF_SIZE];

static int buf_idx = 0;

// ================= HELPERS =================

float constrain(float v, float lo, float hi)
{

    return (v < lo) ? lo : (v > hi) ? hi
        : v;
}

float computePID(PID& p, float val)
{
    if (thrusterOff) return 0;
    float error = p.ref - val;

    p.integral += error * dt;

    p.integral = constrain(p.integral, -1, 1);

    float derivative = (error - p.prev) / dt;

    float output = p.kp * error +

        p.ki * p.integral +

        p.kd * derivative;

    p.prev = error;

    return output;
}

void control::init()
{

    thrusterOff = false;

    pidUpdate = false;
}

void control::stbUpdate()
{

    int c;

    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT)
    {

        if (c == '\n' || c == '\r')
        {

            input_buf[buf_idx] = '\0';

            printf("CMD: [%s]\n", input_buf);

            // Thruster control

            if (!pidUpdate && !refUpdate)
            {

                if (strcmp(input_buf, "0") == 0)
                {

                    thrusterOff = true;

                    printf("Thrusters off\n");
                }

                else if (strcmp(input_buf, "1") == 0)
                {

                    thrusterOff = false;

                    printf("Thrusters on\n");
                }

                else if (strcmp(input_buf, "p") == 0 && thrusterOff)
                {

                    pidUpdate = true;

                    printf("Update PID: <%f %f %f>\n", tune->kp, tune->ki, tune->kd);
                }

                else if (strcmp(input_buf, "r") == 0)
                {
                    refUpdate = true;
                    printf("Update ref: <%f>\n", tune->ref);
                }
            }

            else if (pidUpdate & !refUpdate)
            {

                float kp, ki, kd;
                int args = sscanf(input_buf, "%f %f %f", &kp, &ki, &kd);
                if (args == 3)
                {
                    tune->kp = kp;

                    tune->ki = ki;

                    tune->kd = kd;
                    printf("Parsed: %f %f %f\n", tune->kp, tune->ki, tune->kd);

                    pidUpdate = false;
                }
                else if (args == 0) {
                    printf("No update\n");
                    pidUpdate = false;
                }
                else
                {
                    printf("Invalid format\n");
                }
            }

            else if (refUpdate)
            {
                float ref;
                int args = sscanf(input_buf, "%f", &ref);
                if (args == 1)
                {
                    tune->ref = ref;
                    printf("Parsed: %f\n", tune->ref);
                }
                refUpdate = false;
            }

            buf_idx = 0;
        }

        else
        {

            if (buf_idx < BUF_SIZE - 1)
            {

                input_buf[buf_idx++] = (char)c;
            }
        }
    }

    // ---------- OUTER LOOP (ANGLE â†’ Ï‰_ref) ----------

    float wx_ref = computePID(pid_roll, state.roll);

    float wy_ref = computePID(pid_pitch, state.pitch);

    // ---------- INNER LOOP (LQR) ----------

    float omega_err_x = state.wx - wx_ref;

    float omega_err_y = state.wy - wy_ref;

    float tau_roll = -tau_scale * (K_tau[0][0] * omega_err_x + K_tau[0][1] * omega_err_y);

    float tau_pitch = -tau_scale * (K_tau[1][0] * omega_err_x + K_tau[1][1] * omega_err_y);

    // ---------- Z CONTROL ----------

    float z_error = 0.0;

    float Fz_pid = computePID(pid_z, z_error);

    //float Fz = Fz_eq + Fz_pid
    float Fz = Fz_pid;

    // ---------- XtoF MIXING ----------

    float F1 = XtoF[0][0] * tau_roll + XtoF[0][1] * tau_pitch + XtoF[0][2] * Fz;

    float F2 = XtoF[1][0] * tau_roll + XtoF[1][1] * tau_pitch + XtoF[1][2] * Fz;

    float F3 = XtoF[2][0] * tau_roll + XtoF[2][1] * tau_pitch + XtoF[2][2] * Fz;

    // ---------- SATURATION ----------

    F1 = constrain(F1, F_MIN, F_MAX);

    F2 = constrain(F2, F_MIN, F_MAX);

    F3 = constrain(F3, F_MIN, F_MAX);

    if (!thrusterOff)
        // printf("%f\t%f\t\t%f\t%f\t%f\n", state.roll, state.pitch, F1, F2, F3);

    // ---------- LUT â†’ DSHOT ----------

        if (thrusterOff)
        {

            throttle.VL = thrustToDshot(0);

            throttle.VR = thrustToDshot(0);

            throttle.VB = thrustToDshot(0);

            return;
        }
        else
        {

            throttle.VL = thrustToDshot(F3);

            throttle.VR = thrustToDshot(F2);

            throttle.VB = thrustToDshot(F1);
        }
}

void control::navUpdate(float nav_dt) {
    throttle.HL = state.dx + state.dyaw;
    throttle.HR = state.dx - state.dyaw;
}

void control::navStop() {
    throttle.HL = 48;
    throttle.HR = 48;
}