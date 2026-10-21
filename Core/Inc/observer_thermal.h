/*
 * observer_thermal.h
 *
 *  Created on: Sep 29, 2026
 *      Author: kritsada
 */

#ifndef OBSERVER_THERMAL_H_
#define OBSERVER_THERMAL_H_
//https://robot-daycare.com/posts/2019-11-01-motor-temperature-estimation-without-a-sensor/
typedef struct
{

	float qd_in;
	float qd_out;
	float r_niminal;
	float t_ambient;
	float r_th;
	float temperature;
	float resistance;

}obs_thermal_t;


#endif /* OBSERVER_THERMAL_H_ */
