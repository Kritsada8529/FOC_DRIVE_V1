/*
 * DRV8302.h
 *
 *  Created on: May 31, 2025
 *      Author: munir
 */

#ifndef DRV8353_DRIVER_INC_DRV8353_H_
#define DRV8353_DRIVER_INC_DRV8353_H_

#define ADC_REF_VOLTAGE 3.3
#define ADC_RESOLUTION 4095
#define SHUNT_RESISTANCE 0.01
#define CURRENT_AMP_GAIN 10

#include <DRV8353_config.h>

typedef enum {
	_6_PWM_MODE, // the device supports 6 independent PWM inputs
	_3_PWM_MODE // the device must be connected to ONLY 3 PWM input signals on INH_x
}PWM_mode_t;

typedef enum {
	_SHUTDOWN_MODE, // the gate driver will shutdown the channel which detected an over-current event
	_CYCLE_MODE // the gate driver will operate in a cycle-by-cycle current limiting mode
}OC_mode_t;

typedef enum {
	_10VPV,
	_40VPV
}CSA_gain_t;

typedef struct {


	volatile uint32_t *adc_ia;
	volatile uint32_t *adc_ib;
	volatile uint32_t *adc_ic;
	

	volatile uint32_t *pwm_a;
	volatile uint32_t *pwm_b;
	volatile uint32_t *pwm_c;

	uint32_t pwm_freq;
	uint32_t pwm_resolution;

	PWM_mode_t pwm_mode;
	OC_mode_t oc_mode;
	CSA_gain_t gain_mode;

	float R_shunt;
	float v_offset_a;
	float v_offset_b;
	float v_offset_c;
	float v_to_current; //pre-calculate conversion from voltaeg(V) to current(A)
	float gainA,  gainB,  gainC;
	float a;
	float b;
	float c;
	float prev_d;
	float prev_q;
	float lowpass_d;
	float lowpass_q;
	float lowpass_alpha_d;
	float lowpass_alpha_q;
    uint8_t initialized;
	uint8_t offset_ready;

} DRV8353_t;






extern TIM_HandleTypeDef htim1;



#endif /* DRV8302_DRIVER_INC_DRV8302_H_ */
