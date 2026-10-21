/*
 * AS5600.h
 *
 *  Created on: Sep 19, 2026
 *      Author: kritsada
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_

#define DEGREES_PER_REV       360.0f     // For 1:1 gear ratio
#define SENSOR_DIRECTION 1 //(1 or -1)
#define ANGLE_SCALE_FACTOR    (360.0f / 4095.0f)
#define AS5600_ADDRESS (0x36 << 1) // 00110110

typedef struct {
    float prev_angle_rad;
    float full_rotations;
    float now_angle;
    float angle_el;
    float zero_electric_angle;
    int magnet_ok;
    float initial_angle_rad;
    float angle_filtered;
	float prev_angle_filtered;
	float rpm_alpha;
	float prev_angle_rpm;
} MagnetSensorState;

int16_t readTwo_register(I2C_HandleTypeDef *hi2c,uint8_t Sensor_register);
uint8_t  i2c_write_as5600(I2C_HandleTypeDef *hi2c,  uint8_t Sensor_register, uint8_t register_value);
uint8_t  i2c_read_as5600(I2C_HandleTypeDef *hi2c, uint8_t Sensor_register, uint8_t *return_value, uint8_t size_n);
float get_degree_as5600(I2C_HandleTypeDef *hi2c, float gear_ratio);
int16_t get_raw_as5600(I2C_HandleTypeDef *hi2c);
#endif /* INC_AS5600_H_ */
