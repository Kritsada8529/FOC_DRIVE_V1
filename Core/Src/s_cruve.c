/*
 * s_cruve.c
 *
 *  Created on: Oct 18, 2026
 *      Author: kritsada
 */


#include "s_cruve.h"


// Initialize S-curve profile
void SCurve_Init(
    SCurve_t *p,
    float distance_deg,
    float max_rpm,
    float max_acc_rpm_s,
    float jerk_ramp_time)
{
    p->target_deg = fabsf(distance_deg);
    p->pos_deg = 0.0f;
    p->vel_dps = 0.0f;
    p->t = 0.0f;
    p->done = 0;

    // Convert RPM to degree/s
    p->vmax_dps = max_rpm * 6.0f;

    // Convert RPM/s to degree/s^2
    p->amax_dps2 = max_acc_rpm_s * 6.0f;

    p->tj = jerk_ramp_time;

    if (p->tj <= 0.0f)
        p->tj = 0.2f;

    // Jerk limit: deg/s^3
    p->jerk_dps3 = p->amax_dps2 / p->tj;

    float D = p->target_deg;
    float A = p->amax_dps2;
    float tj = p->tj;
    float V = p->vmax_dps;

    // Time to reach maximum velocity
    float Tacc_max = V / A + tj;

    // Distance required to accelerate and decelerate
    float D_required = V * Tacc_max;

    if (D >= D_required)
    {
        // Trapezoidal velocity with S-curve acceleration
        p->t_acc = Tacc_max;
        p->t_cruise = (D - D_required) / V;
    }
    else
    {
        // Short move: triangular velocity profile
        p->t_acc = 0.5f *
            (tj + sqrtf(tj * tj + 4.0f * D / A));

        p->t_cruise = 0.0f;
    }

    float T = p->t_acc;
    p->t_total = 2.0f * T + p->t_cruise;

    // Actual peak velocity is lower for short moves
    if (p->t_cruise == 0.0f)
        p->vmax_dps = A * (T - tj);
}


// Call once per position-loop period
// dt is the position-loop sampling time in seconds
float SCurve_Update(SCurve_t *p, float dt)
{
    if (p->done)
        return p->target_deg;

    float T = p->t_acc;
    float tj = p->tj;
    float A = p->amax_dps2;
    float J = p->jerk_dps3;

    float tc = p->t_cruise;
    float t = p->t;

    float a;

    // Acceleration phase
    if (t < tj)
    {
        a = J * t;
    }
    else if (t < T - tj)
    {
        a = A;
    }
    else if (t < T)
    {
        a = A - J * (t - (T - tj));
    }

    // Constant-speed phase
    else if (t < T + tc)
    {
        a = 0.0f;
    }

    // Deceleration phase
    else if (t < T + tc + tj)
    {
        a = -J * (t - (T + tc));
    }
    else if (t < 2.0f * T + tc - tj)
    {
        a = -A;
    }
    else
    {
        a = -A + J * (t - (2.0f * T + tc - tj));
    }

    // Integrate acceleration to obtain position and velocity
    float old_v = p->vel_dps;

    p->pos_deg += old_v * dt + 0.5f * a * dt * dt;
    p->vel_dps += a * dt;
    p->t += dt;

    if (p->t >= p->t_total)
    {
        p->pos_deg = p->target_deg;
        p->vel_dps = 0.0f;
        p->done = 1;
    }

    return p->pos_deg;
}
