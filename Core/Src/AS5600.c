/*
 * AS5600.c
 *
 *  Created on: Sep 19, 2026
 *      Author: kritsada
 */

#include "stm32f4xx_hal.h"
#include "AS5600.h"


uint8_t  i2c_write_as5600(I2C_HandleTypeDef *hi2c,  uint8_t Sensor_register, uint8_t register_value)
{
	//HAL_StatusTypeDef status = HAL_OK;

if( HAL_I2C_Mem_Write(hi2c,AS5600_ADDRESS , Sensor_register, I2C_MEMADD_SIZE_8BIT, &register_value, 1,10) !=HAL_OK)
{
	return 1;
}
return 0;
}


int16_t readTwo_register(I2C_HandleTypeDef *hi2c, uint8_t Sensor_register)
{
	  uint8_t data_return[2];
	  i2c_read_as5600(hi2c, Sensor_register , data_return, 2 );
      return ( data_return[0] << 8 ) | data_return[1];
}

uint8_t  i2c_read_as5600(I2C_HandleTypeDef *hi2c, uint8_t Sensor_register, uint8_t *return_value, uint8_t size_n)
{
	HAL_StatusTypeDef status;
	//uint8_t txbuf[3]=0;

    if( HAL_I2C_Mem_Read(hi2c,AS5600_ADDRESS ,Sensor_register, I2C_MEMADD_SIZE_8BIT, return_value, size_n,10) != HAL_OK)
    {
    	return 1;
    }

	return 0;

}

float get_degree_as5600(I2C_HandleTypeDef *hi2c, float gear_ratio)
{
	  int16_t data=0;
	  float angle_deg,angle_raw;
	  data =readTwo_register(hi2c, 0x0C);
	  angle_raw =  (float)data*ANGLE_SCALE_FACTOR;

	  //float filtered_diff = angle_raw;
	 // filtered_diff -= 360.0f * floorf((filtered_diff + 180.0f) / 360.0f);


     return    angle_raw;
}

int16_t get_raw_as5600(I2C_HandleTypeDef *hi2c)
{
	 int16_t rawdata=0;
	 rawdata=readTwo_register(hi2c,0x0C);
	 return rawdata;
}


