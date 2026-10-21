/*
 * s_cruve.h
 *
 *  Created on: Oct 18, 2026
 *      Author: kritsada
 */

#ifndef INC_S_CRUVE_H_
#define INC_S_CRUVE_H_
#include <stdint.h>
typedef struct
{
    float target_deg;
    float pos_deg;
    float vel_dps;

    float vmax_dps;
    float amax_dps2;
    float jerk_dps3;
    float tj;

    float t_acc;
    float t_cruise;
    float t_total;
    float t;

    uint8_t done;
} SCurve_t;
float SCurve_Update(SCurve_t *p, float dt);
void SCurve_Init(
    SCurve_t *p,
    float distance_deg,
    float max_rpm,
    float max_acc_rpm_s,
    float jerk_ramp_time);

#endif /* INC_S_CRUVE_H_ */
