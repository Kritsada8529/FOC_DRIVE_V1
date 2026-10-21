/*
 * FOC_utils.h
 *
 *  Created on: May 31, 2025
 *      Author: munir
 */

#ifndef FOC_INC_FOC_UTILS_H_
#define FOC_INC_FOC_UTILS_H_

#include <stdint.h>
#include "FOC_config.h"
#include "FOC_math.h"
#include "AS5600.h"
#include "pid_utils.h"



#define ERROR_LUT_SIZE (1024)

#define MAG_CAL_RES (1024*2)
#define MAG_CAL_STEP ((TWO_PI * POLE_PAIR) / (float)MAG_CAL_RES)

#define is_foc_ready() (foc_ready)
#define foc_reset_flag() (foc_ready = 0)
#define foc_set_flag() (foc_ready = 1)

/* extern variable */
extern _Bool foc_ready;
float Vd_buff[MAX_I_SAMPLE];
float Vq_buff[MAX_I_SAMPLE];
float Id_buff[MAX_I_SAMPLE];
float Iq_buff[MAX_I_SAMPLE];

#if DEBUG_HFI
extern float param1_debug_buff[MAX_SAMPLE_BUFF];
extern float param2_debug_buff[MAX_SAMPLE_BUFF];
extern float param3_debug_buff[MAX_SAMPLE_BUFF];
extern float param4_debug_buff[MAX_SAMPLE_BUFF];
#endif


extern I2C_HandleTypeDef hi2c2;

typedef enum {
	TORQUE_CONTROL_MODE,
	SPEED_CONTROL_MODE,
	POSITION_CONTROL_MODE,
	CALIBRATION_MODE,
	CALIBRATION_RS_L_MODE,
	AUDIO_MODE,
	IDEL_MODE,
}motor_mode_t;

typedef enum {
	FOC_MODE_SENSORED,
	FOC_MODE_SENSORLESS_SMO_HFI,
	FOC_MODE_SENSORLESS_SMO_HFI_NEW,
	FOC_MODE_HYBRID
}foc_mode_t;

typedef enum {
	NORMAL_DIR, REVERSE_DIR
}dir_mode_t;

typedef enum {
  RS, LD, LQ
}inject_taregt_t;

// state machine for HFI
typedef enum {
	MOTOR_STATE_HFI,
	MOTOR_STATE_SMO,
	MOTOR_STATE_SMO_TO_HFI,
	MOTOR_STATE_SENSORED,
}motor_state_t;

// state machine for polarity detection
typedef enum {
	P_DET_START,
	P_DET_POSITIVE,
	P_DET_WAITING_POSITIVE,
	P_DET_NEGATIVE,
	P_DET_WAITING_NEGATIVE,
	P_DET_STOP
}p_det_state_t;

typedef struct {
	//DRV8302_t drv8302;
	DRV8353_t drv8353;
	MagnetSensorState as5600;
	foc_mode_t foc_mode;

	uint8_t pole_pairs;
	float kv;
	float Rs;
	float Ld;
	float Lq;
	float max_current;
	float flux_linkage;

	float meas_inj_freq;
	float meas_inj_amp;
	float meas_inj_omega;
	inject_taregt_t meas_inj_target;
	int meas_inj_n;
	_Bool meas_inj_start_flag;
    float m_angle_deg;
	float m_angle_rad; // mechanical angle
	float e_angle_rad; // electrical angle
	float e_angle_rad_comp; // electrical angle
	float m_angle_offset;
	float e_angle_offset;
	float e_rad;
	float last_e_rad;
	float e_omega;

	float vd, vq;
	float id, iq;
	float id_filtered, iq_filtered;
	float v_alpha, v_beta;
	float i_alpha, i_beta;
	float va, vb, vc;
	float ia, ib, ic;
	float v_bus;
	float i_bus;
	float da, db, dc;

	float rpm_temp;
	float actual_rpm;
	float actual_angle_rad;
	int32_t m_angle_overflow_count;

	float I_ctrl_bandwidth;
	float Is_ref;
	float id_ref, iq_ref;
	float rpm_ref;

    uint8_t loop_count;

	PID_Controller_t id_ctrl, iq_ctrl;
	PID_Controller_t speed_ctrl;
	PID_Controller_t pos_ctrl;

	//field weakening
	//PID_Controller_t fw_ctrl;
	float fw_vs_ref;
	_Bool fw_enable;

	motor_mode_t control_mode;

	float gear_ratio;
	dir_mode_t sensor_dir;

	motor_state_t state;


	//polarity detection
	p_det_state_t pd_state;
	float pd_v_pulse;
	float pd_i_p;
	float pd_i_n;
	uint16_t pd_time;
	uint16_t pd_count;

	//debug
	int sample_index;
	_Bool collect_sample_flag;
}foc_t;



#endif /* FOC_INC_FOC_UTILS_H_ */
