/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "AS5600.h"
#include "DRV8353.h"
#include "FOC_math.h"
#include "FOC_config.h"
#include "FOC_utils.h"
#include "pid_utils.h"
#include "bldc_midi.h"
#include "sliding_mode_observer.h"
#include "s_cruve.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// #define BLDC_PWM_FREQ AUDIO_SAMPLE_RATE
#define R_SHUNT	0.01f
#define V_OFFSET_A	1.645f
#define V_OFFSET_B	1.657f
#define POLE_PAIR	(7)
#define SVPWM_FREQUENCY_HZ 5000.0f
#define DT (1.0f / SVPWM_FREQUENCY_HZ)
#define INTEGRAL_D_MAX 5.0f
#define INTEGRAL_Q_MAX 5.0f
#define VOLTAGE_ALIGN  1.0f
#define VOLTAGE_POWER_SUPPLY 12.0f

#define CAL_TORQUE 0
#define CAL_SPEED  1
#define CAL_POSITION 2
#define CAL_TORQUE_SPEED_POSITION 3





#define VOLTAGE_LIMIT  (VOLTAGE_POWER_SUPPLY * 0.866f)
#define _PI       3.14159265358979323846f
#define _3PI_2    (1.5f * _PI)

#define TEST_W_RAD   5.0f   // 電�?検査�?�5
#define TEST_UQ_V      1.0f   // q 軸電圧 [V]

#define _constrain(x, min_val, max_val)  ( ((x)<(min_val)) ? (min_val) : (((x)>(max_val)) ? (max_val) : (x)) )

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

foc_t hfoc;
smo_t hsmo;
SCurve_t motion;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void DRV8301_CS_Deselect(void);
void DRV8301_CS_Select(void);
uint16_t spi_read_DRV8301(uint8_t  reg);
int torque_control(foc_t *hfoc);
void spi_write_DRV8301(uint8_t  reg, uint16_t  value);
void svpwm_test(void);
void open_loop_voltage_control(foc_t *hfoc, float vd_ref, float vq_ref, float angle_rad);

static void control_init(void) ;

void myfoc_cal_encoder_misalignment(foc_t *hfoc);

//---------------------------------------------------//
int8_t foc_sensor_cal_elec_angle(foc_t *hfoc, I2C_HandleTypeDef *hi2c);
int8_t foc_init(foc_t *hfoc, I2C_HandleTypeDef *hi2c);
void CalibrateZeroElectricAngle(foc_t *hfoc, I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim);

void Update_Magnet_Angle(foc_t *hfoc, I2C_HandleTypeDef *hi2c);
void InitMagnetSensorState(foc_t *hfoc, I2C_HandleTypeDef *hi2c);
float ADC_to_I(int32_t adc_diff);
float Convert_ADC_to_Current(uint32_t adc_value);
void setPhaseVoltage(foc_t *hfoc, float angle_el, TIM_HandleTypeDef *htim);
void PI_current_dq(foc_t *hfoc);
void PI_current_dq_debug(foc_t *hfoc);

void LPF_current_dq(foc_t *hfoc);
void getDQCurrents(foc_t *hfoc, float angle_el);
void FOC_FastLoop(void);
int8_t FOC_Torque_Control(void);
 void FOC_OpenLoopSensorTest(void);
 float get_rpm_as5600(foc_t *hfoc);
 void foc_speed_control_update(foc_t *hfoc, float _rpm_error) ;
 void foc_position_control_update(foc_t *hfoc, float deg_reference) ;
 void audio_loop(foc_t *hfoc, TIM_HandleTypeDef *htim);
 static float note_freq(int note_number);
  uint16_t audio_test(foc_t *hfoc);
  void meas_inj_dq_process(foc_t *hfoc, float ts);
  static void calibration_seq(void);
  void estimate_inductance(foc_t *hfoc, float ts);
  void estimate_resistance(foc_t *hfoc);
  static int measure_L(float f, float amp);
  static int measure_R(float vdc);
  static void start_measure(inject_taregt_t target);
  int8_t relay_autotune(foc_t *hfoc, float delta_ts, float input, uint8_t type_pid);
 uint8_t speed_control(float input_rpm);
 void cal_auto_torque_speed(void);
 
void cal_pid_relay(void);


void  calculate_Trapezoidal_profile(
    float go_angle, float max_rpm_speed, float max_rpm_acc, float ts, 
    float *step_ramp, uint32_t *cnt_ramp,
    float *step_cruise, uint32_t *cnt_cuise);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t cnt=0;
uint8_t is_send=1;
uint32_t adc_ia,adc_ib,adc_ic;
uint8_t sendbuffer[128];
int16_t data,is_i2c_ready=0;
float angle_deg=0;

float  sp_input=0.0f;
float _rpm_error=0.0f;
float _pos_error=0.0f;

uint8_t start_cal = 0;
uint8_t cnt_cal=0;
float sp_rpm = 0.0f;
float present_pos=0.00f;
uint16_t counter_relay=0;
uint16_t counter_relay_max=0;
uint16_t cnt_start_up_speed=0;


float Iq_buff_relay[1024*2];//2k

uint16_t cnt_relay=0;

uint16_t loop_start_speed=0;
uint16_t lock_start=0;
uint8_t mode_cal_pid=0;
uint8_t cal_auto=0;
uint8_t cal_auto_step=0;

 uint16_t lock_speed_start=0;
 float lpf_speed_prv=0.0;
 float speed_relay=0.0;
 uint16_t end_pos=0;
static float sp_pos=0.0f;

float step_ramp;
uint32_t cnt_ramp;
float step_cruise; 
uint32_t cnt_cuise;





void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	is_send=1;
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
;

}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if (hadc->Instance == ADC1) {
		 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10, GPIO_PIN_SET);
	uint32_t rawA = (int32_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    uint32_t rawB = (int32_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    uint32_t rawC = (int32_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

    static uint32_t cnt = 0;
    static float sumA = 0, sumB = 0, sumC = 0;
    const uint32_t CAL_SAMPLES = 2000;

    if(!hfoc.drv8353.offset_ready)
    {
    	 sumA += rawA;
    	 sumB += rawB;
    	 sumC += rawC;
    	 if(++cnt == CAL_SAMPLES)
    	 {
    		 hfoc.drv8353.v_offset_a= sumA/ CAL_SAMPLES;
    		 hfoc.drv8353.v_offset_b= sumA/ CAL_SAMPLES;
    		 hfoc.drv8353.v_offset_c= sumA/ CAL_SAMPLES;
    		 hfoc.drv8353.offset_ready=1;

    	 }
    	 return;
    }

    float ia = ADC_to_I(rawA -  hfoc.drv8353.v_offset_a);
    float ib = ADC_to_I(rawB -  hfoc.drv8353.v_offset_b);
    float ic = ADC_to_I(rawC -  hfoc.drv8353.v_offset_c);

    ia *=hfoc.drv8353.gainA;
    ib *=hfoc.drv8353.gainB;
    ic *=hfoc.drv8353.gainC;

    float mid = (ia + ib + ic) * (1.0f / 3.0f);

    hfoc.drv8353.a =  ia - mid;
    hfoc.drv8353.b =  ib - mid;
    hfoc.drv8353.c =  ic - mid;
	//printf("%.3f,%.3f,%.3f\r\n", hfoc.drv8353.a, hfoc.drv8353.b, hfoc.drv8353.c);
    FOC_FastLoop();
   //  FOC_OpenLoopSensorTest();

    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10, GPIO_PIN_RESET);
  }
	if (hadc->Instance == ADC2) {
  }
	if (hadc->Instance == ADC3) {

	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    float bandwid_speed=5.0f; //10Hz
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
   i2c_write_as5600(&hi2c2,  0x08, 0x20);
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // Starts CH1
   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); // Starts CH1N
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);    // Starts CH2
   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); // Starts CH2N
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);   // Starts CH3
   HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); // Starts CH3N
   HAL_Delay(100);
  //data =readTwo_register(&hi2c2, 0x0E);
   hfoc.drv8353.pwm_resolution=8999;
   hfoc.v_bus=12.0;
   HAL_GPIO_WritePin(GPIOB, EN_GATE_Pin, GPIO_PIN_RESET);
   init_trig_lut();

   // current sensor

   HAL_GPIO_WritePin(GPIOB, EN_GATE_Pin, GPIO_PIN_SET);

   HAL_GPIO_WritePin(GPIOB, DC_CAL_Pin, GPIO_PIN_SET);

   HAL_Delay(1000);
   HAL_GPIO_WritePin(GPIOB, DC_CAL_Pin, GPIO_PIN_RESET);

   HAL_GPIO_WritePin(CS_SPI2_GPIO_Port, CS_SPI2_Pin, GPIO_PIN_SET);

   spi_write_DRV8301(0x03, 0x400); //100 0000 0000  Limit Vds =0.155V@25A, 10mV/V


  // DRV8353_ADC_config(&hfoc.drv8353, &(ADC1->JDR1), &(ADC1->JDR2));
  // DRV8353_current_sens_config(&hfoc.drv8353, _10VPV, R_SHUNT, V_OFFSET_A, V_OFFSET_B);
  foc_init(&hfoc,&hi2c2);

  float Rs=3.325445;
  float Ls= 0.0027429;
  smo_init(&hsmo, Rs,  Ls, hfoc.pole_pairs, FOC_TS);

   //foc_gear_reducer_init(&hfoc, 1.0f);
  // foc_set_limit_current(&hfoc, 20.0);
   //foc_motor_init(&hfoc, POLE_PAIR, 360.0f);
   //foc_sensor_init(&hfoc, m_config.encd_offset, REVERSE_DIR);

  /*
   calculate_Trapezoidal_profile(
	36000.0, 6000.0, 2000.0, POSITION_TS,
	 &step_ramp, &cnt_ramp,
	 &step_cruise, &cnt_cuise);
   */


   SCurve_Init(
           &motion,
           360.0f,  // Target angle (degree)
           300.0f,   // Maximum speed (RPM)
           10.0f,    // Maximum acceleration (RPM/s)
           0.8f      // Acceleration ramp time (s)
       );

	 HAL_Delay(1000);

   InitMagnetSensorState(&hfoc, &hi2c2);
   CalibrateZeroElectricAngle(&hfoc, &hi2c2, &htim1);


     // default mode
  // hfoc.control_mode= CALIBRATION_MODE;
 // hfoc.control_mode= CALIBRATION_RS_L_MODE;
 //  hfoc.control_mode=IDEL_MODE;
  hfoc.control_mode=POSITION_CONTROL_MODE;
 //   hfoc.control_mode=SPEED_CONTROL_MODE;
 // hfoc.control_mode=TORQUE_CONTROL_MODE;

   mode_cal_pid = CAL_POSITION;
   cal_auto =0; //0 =Off , 1 = On
  
   hfoc.foc_mode = FOC_MODE_SENSORED;
   
   hfoc.id_ref=0.0f;
   hfoc.iq_ref=0.0f;
   
if(hfoc.control_mode==CALIBRATION_MODE)
{
  if( mode_cal_pid == CAL_TORQUE  &&  !cal_auto)
  	{
	hfoc.id_ctrl.kp=10.0f;
	hfoc.id_ctrl.ki=0.0f;
	hfoc.iq_ctrl.kp=10.0f;
	hfoc.iq_ctrl.ki=0.0f;
  	}
  else if( mode_cal_pid == CAL_SPEED &&  !cal_auto)
  	{

	  sp_input=0.0f;//speed RPM
     //hfoc.id_ctrl.kp=1.23057;
	 //hfoc.id_ctrl.ki=28.5384;
	 //hfoc.iq_ctrl.kp=1.23057;
	 //hfoc.iq_ctrl.ki=28.5384;

	  //hfoc.id_ctrl.kp=1.305710;
	  //hfoc.id_ctrl.ki=28.810896;
	 // hfoc.iq_ctrl.kp=1.305710;
	 // hfoc.iq_ctrl.ki=28.810896;
	 
	  hfoc.id_ctrl.kp= 0.844950;
	  hfoc.id_ctrl.ki=18.644093;
	  hfoc.iq_ctrl.kp=0.844950;
	  hfoc.iq_ctrl.ki=18.644093;


	  hfoc.speed_ctrl.e_deadband=0.0;
      hfoc.speed_ctrl.out_max= 5.0;
      hfoc.speed_ctrl.out_min= -5.0;
      hfoc.speed_ctrl.kp=0.000f;
      hfoc.speed_ctrl.ki=0.00f;
      hfoc.speed_ctrl.integral=0.0f;
      hfoc.speed_ctrl.ts=SPEED_TS;
      hfoc.as5600.rpm_alpha=0.001;

      printf("CAL SPEED\r\n");
  	}

  else if( mode_cal_pid == CAL_POSITION  &&  !cal_auto)
    	{
      

	  // sp_input = RAD_TO_DEG(hfoc.as5600.initial_angle_rad);
	   sp_input = 2.5f;
	   printf("ANGLE CAL (DEG): =%.6f\r\n", sp_input);

  	 // hfoc.id_ctrl.kp=1.305710;
  	 // hfoc.id_ctrl.ki=28.810896;
  	 // hfoc.iq_ctrl.kp=1.305710;
  	 // hfoc.iq_ctrl.ki=28.810896;

	   
	   hfoc.id_ctrl.kp= 0.844950;
	  hfoc.id_ctrl.ki=18.644093;
	  hfoc.iq_ctrl.kp=0.844950;
	  hfoc.iq_ctrl.ki=18.644093;


  	   hfoc.speed_ctrl.e_deadband=0.0;
       hfoc.speed_ctrl.out_max= 5.0;
       hfoc.speed_ctrl.out_min= -5.0;

      // hfoc.speed_ctrl.kp=  0.006792f;
	  // hfoc.speed_ctrl.ki=  0.617494;

	   hfoc.speed_ctrl.kp=  0.010863;
	   hfoc.speed_ctrl.ki=  0.548649;




	  
	   
		hfoc.pos_ctrl.kp=0.0f;
		hfoc.pos_ctrl.ki=0.0f;
		hfoc.pos_ctrl.kd=0.0f;


        hfoc.speed_ctrl.integral=0.0f;
        hfoc.speed_ctrl.ts=SPEED_TS;
        hfoc.as5600.rpm_alpha=0.001;

        printf("CAL POSITION angle %.6f\r\n", sp_input);
    	}

  
  
   if(cal_auto==1)
  	{
  	   mode_cal_pid=CAL_TORQUE_SPEED_POSITION;
       hfoc.id_ctrl.kp=10.0f;
	   hfoc.id_ctrl.ki=0.0f;
	   hfoc.iq_ctrl.kp=10.0f;
	   hfoc.iq_ctrl.ki=0.0f;

	  hfoc.speed_ctrl.e_deadband=0.0;
      hfoc.speed_ctrl.out_max= 5.0;
      hfoc.speed_ctrl.out_min= -5.0;
	  
      hfoc.speed_ctrl.kp=0.000f;
      hfoc.speed_ctrl.ki=0.00f;
	  
      hfoc.speed_ctrl.integral=0.0f;
      hfoc.speed_ctrl.ts=SPEED_TS;
      hfoc.as5600.rpm_alpha=0.001;

	  hfoc.pos_ctrl.kp=0.0f;
      hfoc.pos_ctrl.ki=0.0f;
      hfoc.pos_ctrl.kd=0.0f;
   


     hfoc.pos_ctrl.integral=0.0f;
     hfoc.pos_ctrl.ts=POSITION_TS;
     hfoc.pos_ctrl.d_alpha_filter=0.45;
     hfoc.pos_ctrl.d_max=10;

	  printf("CAL TORQUE+SPEED+POSITION \r\n");

  	}

	

}


if(hfoc.control_mode==TORQUE_CONTROL_MODE)
{

    hfoc.iq_ref=-0.2f;

 // hfoc.id_ctrl.kp=3.905f;
//  hfoc.id_ctrl.ki=25.478f;
 // hfoc.iq_ctrl.kp=3.905f;
//  hfoc.iq_ctrl.ki=25.478f;
    // Relay Control 1
	//hfoc.id_ctrl.kp=0.65f;
	//hfoc.id_ctrl.ki=3.78f;
	//hfoc.iq_ctrl.kp=0.65f;
	//hfoc.iq_ctrl.ki=3.78f;
	 // Relay Control 2 OK
	//hfoc.id_ctrl.kp=1.840396614;
	//hfoc.id_ctrl.ki=109.3304919;
	//hfoc.iq_ctrl.kp=1.840396614;
	//hfoc.iq_ctrl.ki=109.3304919;
	 // Relay Control 3 
	// hfoc.id_ctrl.kp=1.23057;
	// hfoc.id_ctrl.ki=28.5384;
	// hfoc.iq_ctrl.kp=1.23057;
	// hfoc.iq_ctrl.ki=28.5384;
	  //Relay Control 4
	  
	  hfoc.id_ctrl.kp= 0.844950;
	  hfoc.id_ctrl.ki=18.644093;
	  hfoc.iq_ctrl.kp=0.844950;
	  hfoc.iq_ctrl.ki=18.644093;


	
	

}
 
 //  test auto parameter RS, Id and Iq : for speen conrtol is OK
 if( hfoc.control_mode==SPEED_CONTROL_MODE)
 {
    /*
	// PI =OK  1
 	hfoc.id_ctrl.kp=4.730579457;
	hfoc.id_ctrl.ki=0.125;
	hfoc.iq_ctrl.kp=4.730579457;
	hfoc.iq_ctrl.ki=0.125;
	
	hfoc.speed_ctrl.kp=0.000012625f;
    hfoc.speed_ctrl.ki=0.2;//0.2f;//0.00010f;

	*/
	
	// PI =OK  2 GOOD
	/*	hfoc.id_ctrl.kp=1.23057;
		hfoc.id_ctrl.ki=28.5384;
		hfoc.iq_ctrl.kp=1.23057;
		hfoc.iq_ctrl.ki=28.5384;
		
		//hfoc.speed_ctrl.kp=0.0012625f;
		//hfoc.speed_ctrl.ki=1.02;
		hfoc.speed_ctrl.kp=0.003617;
		hfoc.speed_ctrl.ki=1.096321;
		*/

	
	// PI =OK  --- 3 GOOD
	/*
	   hfoc.id_ctrl.kp=1.305710;
	   hfoc.id_ctrl.ki=28.810896;
	   hfoc.iq_ctrl.kp=1.305710;
	   hfoc.iq_ctrl.ki=28.810896;
	
	   hfoc.speed_ctrl.kp=0.014748;
		hfoc.speed_ctrl.ki=0.558625;
		*/

	// PI =OK  --- 4 GOOD
	/*
	   hfoc.id_ctrl.kp=1.305710;
	   hfoc.id_ctrl.ki=28.810896;
	   hfoc.iq_ctrl.kp=1.305710;
	   hfoc.iq_ctrl.ki=28.810896;
	
	   hfoc.speed_ctrl.kp= 0.006787;
	   hfoc.speed_ctrl.ki=0.237304;
	   */

	 //PI =OK --  5 GOOD
	   
	  hfoc.id_ctrl.kp= 0.844950;
	  hfoc.id_ctrl.ki=18.644093;
	  hfoc.iq_ctrl.kp=0.844950;
	  hfoc.iq_ctrl.ki=18.644093;

	  hfoc.speed_ctrl.kp=  0.006792f;
	  hfoc.speed_ctrl.ki=  0.617494;
		
	
    hfoc.speed_ctrl.e_deadband=0.0;
    hfoc.speed_ctrl.out_max= 5.0;
    hfoc.speed_ctrl.out_min= -5.0;
 
    hfoc.speed_ctrl.integral=0.0f;
    hfoc.speed_ctrl.ts=SPEED_TS;
   
	
 }
   hfoc.id_ctrl.integral=0.0f;
   hfoc.iq_ctrl.integral=0.0f;

   
   
   
   // for position conrtol is OK
   if( hfoc.control_mode==POSITION_CONTROL_MODE)
   	{
    //hfoc.id_ctrl.kp=3.905f;
    //hfoc.id_ctrl.ki=25.478f;
    //hfoc.iq_ctrl.kp=3.905f;
    //hfoc.iq_ctrl.ki=25.478f;

	//hfoc.id_ctrl.kp=4.730579457;
	//hfoc.id_ctrl.ki=0.125;
	//hfoc.iq_ctrl.kp=4.730579457;
	//hfoc.iq_ctrl.ki=0.125;

	// hfoc.id_ctrl.kp=1.23057;
	// hfoc.id_ctrl.ki=28.5384;
	// hfoc.iq_ctrl.kp=1.23057;
	// hfoc.iq_ctrl.ki=28.5384;
	   sp_input =0.0; //RAD_TO_DEG(hfoc.as5600.initial_angle_rad);
	
	// hfoc.id_ctrl.kp=1.305710;
	// hfoc.id_ctrl.ki=28.810896;
	// hfoc.iq_ctrl.kp=1.305710;
	// hfoc.iq_ctrl.ki=28.810896;

	 hfoc.id_ctrl.kp= 0.844950;
	 hfoc.id_ctrl.ki=18.644093;
	 hfoc.iq_ctrl.kp=0.844950;
	 hfoc.iq_ctrl.ki=18.644093;

	// hfoc.id_ctrl.kp= 0.827278;
	 //hfoc.id_ctrl.ki=18.254147;
	// hfoc.iq_ctrl.kp=0.827278;
	 //hfoc.iq_ctrl.ki=18.254147;

 
   hfoc.id_ctrl.integral=0.0f;
   hfoc.iq_ctrl.integral=0.0f;

   hfoc.speed_ctrl.e_deadband=0.0;
   hfoc.speed_ctrl.out_max= 5.0;
   hfoc.speed_ctrl.out_min= -5.0;

   //hfoc.speed_ctrl.kp=2.85f;
   //hfoc.speed_ctrl.ki=0.75f;
  
  // hfoc.speed_ctrl.kp= 8.5431;
   //hfoc.speed_ctrl.ki=0.308567;
   


   // hfoc.speed_ctrl.kp= 0.006787;
	//hfoc.speed_ctrl.ki=0.237304;

   // hfoc.speed_ctrl.kp=  0.006792f;
	//hfoc.speed_ctrl.ki=  0.617494;
	
	 hfoc.speed_ctrl.kp=   0.013577;
	 hfoc.speed_ctrl.ki=  1.542897;
	



   hfoc.speed_ctrl.integral=0.0f;
   hfoc.speed_ctrl.ts=SPEED_TS;

   hfoc.pos_ctrl.e_deadband=0.0;
   hfoc.pos_ctrl.out_max = 1000.0f;
   hfoc.pos_ctrl.out_min= -1000.0f;

   //hfoc.pos_ctrl.kp=0.7225;
   //hfoc.pos_ctrl.ki=0.535f;
   //hfoc.pos_ctrl.kd=0.0045f;

//   hfoc.pos_ctrl.kp=426.163208;
 //  hfoc.pos_ctrl.ki=3026.727783;
//   hfoc.pos_ctrl.kd=4.328454;
   
  // hfoc.pos_ctrl.kp=329.391602;
  // hfoc.pos_ctrl.ki=2495.391113;
   //hfoc.pos_ctrl.kd=3.136467;
   
   // hfoc.pos_ctrl.kp=416.744812;
   // hfoc.pos_ctrl.ki=3157.157959;
   // hfoc.pos_ctrl.kd=3.968244;


  //  hfoc.pos_ctrl.kp=399.104858;
  //  hfoc.pos_ctrl.ki=2519.601318;
  //  hfoc.pos_ctrl.kd=4.560331;

   //  hfoc.pos_ctrl.kp= 436.016296;
   //  hfoc.pos_ctrl.ki=4128.942383;
   //  hfoc.pos_ctrl.kd=3.321398;

     hfoc.pos_ctrl.kp=   423.768616;
     hfoc.pos_ctrl.ki=3210.368652;
     hfoc.pos_ctrl.kd=  4.035124;

     
    
   
 



   hfoc.pos_ctrl.integral=0.0f;
   hfoc.pos_ctrl.ts=POSITION_TS;
   hfoc.pos_ctrl.d_alpha_filter=0.45;
   hfoc.pos_ctrl.d_max=10;
   
   
   
   }
    
   

   hfoc.drv8353.gainA=0.998f;
   hfoc.drv8353.gainB=0.998f;
   hfoc.drv8353.gainC=0.998f;


   hfoc.drv8353.lowpass_alpha_d = 0.01;
   hfoc.drv8353.lowpass_alpha_q = 0.01;

   hfoc.drv8353.initialized=0;
   hfoc.drv8353.offset_ready=0; //start cal offset curent phase
   
   //// hfoc.as5600.rpm_alpha =1  will fillter nois is low
    if(hfoc.control_mode==SPEED_CONTROL_MODE)
    	{
		   //speed control
		    hfoc.as5600.rpm_alpha=0.001;
    	}
	else{
          //position control
           hfoc.as5600.rpm_alpha=0.001;
		}

    hfoc.meas_inj_start_flag = 0; //cal Rs,Ld, Lq

   

 //  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1);
 //  printf(">> Period: %lu\r\n", period);
   //myfoc_cal_encoder_misalignment(&hfoc);


 
   HAL_ADCEx_InjectedStart_IT(&hadc1);
   //printf("TEST FOC ON STM32F4\r\n");
  // HAL_GPIO_WritePin(GPIOB, EN_GATE_Pin, GPIO_PIN_RESET);
   HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//  printf(">> angle_el:%.3f, offset=%.3f raw=%.3f\r\n",  hfoc.as5600.angle_el, hfoc.as5600.zero_electric_angle, hfoc.as5600.prev_angle_rad);
	// float diff_ang = hfoc.m_angle_deg-hfoc.m_angle_offset;

 if(hfoc.control_mode==POSITION_CONTROL_MODE)
 	{
	    
     
      

      uint8_t len_str=sprintf(sendbuffer,"%.3f, %.3f, %.3f, %.6f\r\n",  hfoc.rpm_ref, RAD_TO_DEG(hfoc.actual_angle_rad - hfoc.as5600.initial_angle_rad), hfoc.pos_ctrl.integral,_pos_error);

	  HAL_UART_Transmit_DMA(&huart2, sendbuffer, len_str);

 	}

 if(hfoc.control_mode==SPEED_CONTROL_MODE)
	{
	// sp_input=RAD_TO_DEG(hfoc.actual_angle_rad);//speed RPM  
	 sp_input=300.0f;
	 if(is_send==1)
	  {
	  uint8_t len_str=sprintf(sendbuffer,"%.6f, %.6f, %.6f, %.6f, %.3f, %.6f, %.6f\r\n",
	  	hfoc.actual_rpm, hfoc.iq_ref,  hfoc.speed_ctrl.integral, _rpm_error, sp_input , hfoc.as5600.angle_el, hsmo.e_theta);

	  HAL_UART_Transmit_DMA(&huart2, sendbuffer, len_str);
	  is_send=0;
	  }
	       
			
  }
  if(hfoc.control_mode==TORQUE_CONTROL_MODE)
  {
	if(is_send==1)
	  {
	  uint8_t len_str=sprintf(sendbuffer," Id=  %.6f, %.3f <> Iq=%.6f, %.3f : RPM=%.3f, e_act:%.6f ,e_est:%.6f\r\n", 
	  	hfoc.drv8353.lowpass_d,hfoc.vd,hfoc.drv8353.lowpass_q,hfoc.vq,hfoc.actual_rpm , hfoc.as5600.angle_el,hsmo.e_theta);

	  HAL_UART_Transmit_DMA(&huart2, sendbuffer, len_str);
	  is_send=0;
	  }
      
	
   }
			
	  if (hfoc.control_mode == CALIBRATION_MODE) {
	   #define DEBUG_CAL   1
	   #if  DEBUG_CAL == 0
	    // sp_input=hfoc.actual_angle_rad;//
         if (start_cal == 1) {
             HAL_GPIO_WritePin(GPIOB, EN_GATE_Pin, GPIO_PIN_RESET);
			  for(uint16_t i=0; i < cnt_relay;i++){		  
				 printf("%.3f,%.6f\r\n", sp_input, Iq_buff_relay[i]);
			  	}

			    start_cal =2;
			    cnt_relay=0;
				counter_relay_max=0;
         	}
		  
			  
	   #else
			  
			
	   
			  if (start_cal == 1) {
			  	

				  if(mode_cal_pid==CAL_TORQUE)
				  relay_autotune(&hfoc, FOC_TS, fabsf( hfoc.iq_ref), TORQUE_CONTROL_MODE);
				  else if(mode_cal_pid==CAL_SPEED)
					{
					 relay_autotune(&hfoc, SPEED_TS,  fabsf( hfoc.iq_ref), SPEED_CONTROL_MODE);
					}
				  else if(mode_cal_pid==CAL_POSITION)
					{
					 relay_autotune(&hfoc, POSITION_TS, fabsf( speed_relay), POSITION_CONTROL_MODE);
					}

				 
				   cnt_relay=0;
				   counter_relay_max=0;
				   cal_auto_torque_speed();
			 
			  }

       #endif
	   
			  
	     }



	  if(hfoc.control_mode==IDEL_MODE)
	  {
	     /*  Update_Magnet_Angle(&hfoc, &hi2c2);
           hfoc.actual_rpm= get_rpm_as5600(&hfoc);
		   HAL_Delay(100);
		  if(is_send==1)
		  {
		  uint8_t len_str=sprintf(sendbuffer," RPM =  %.6f  ANGLE = %.3f\r\n",hfoc.actual_rpm ,RAD_TO_DEG(hfoc.actual_angle_rad));

		  HAL_UART_Transmit_DMA(&huart2, sendbuffer, len_str);
		  HAL_GPIO_WritePin(GPIOB, EN_GATE_Pin, GPIO_PIN_RESET);
		  is_send=0;
		  }*/
		  ;
	 }


	  if(hfoc.control_mode== CALIBRATION_RS_L_MODE)

	  	{
         if(start_cal==0)
         	{
		 calibration_seq();
		 HAL_GPIO_WritePin(GPIOB, EN_GATE_Pin, GPIO_PIN_RESET);
		 start_cal=2;
         	}

	  	}

	  
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = 3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 8999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 5;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CS_SPI2_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DC_CAL_Pin|EN_GATE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_SPI2_Pin */
  GPIO_InitStruct.Pin = CS_SPI2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CS_SPI2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_CAL_Pin EN_GATE_Pin */
  GPIO_InitStruct.Pin = DC_CAL_Pin|EN_GATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE intfputc(int ch, FILE *f)
#endif/* __GNUC__ */
/**
* @brief Retargets the C library printf function to the USART.
* @param None
* @retvalNone
*/
PUTCHAR_PROTOTYPE
{
/* Place your implementation of fputc here */
/* e.g. write a character to the EVAL_COM1 and Loop until the end of
transmission */
HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
return ch;

}


void svpwm_test(void)
{
	//open_loop_voltage_control(&hfoc, VD_CAL, VQ_CAL, 0.0f);
	//HAL_Delay(500);

	for(uint16_t i=0; i< ERROR_LUT_SIZE; i++)
	{
      float mech_deg =(float)i*(360.0f/(float)ERROR_LUT_SIZE);
      float elec_deg = mech_deg * POLE_PAIR;
      open_loop_voltage_control(&hfoc, VD_CAL, VQ_CAL, DEG_TO_RAD(elec_deg));
      HAL_Delay(3);
     // float ang=get_degree_as5600(&hi2c2, 1.0);
     // cnt++;
      //uint8_t len_str=sprintf(sendbuffer,"%.2f\r\n",ang);
     // HAL_UART_Transmit(&huart2, sendbuffer, len_str, 1000);

	}
    cnt=0;



}

 
void open_loop_voltage_control(foc_t *hfoc, float vd_ref, float vq_ref, float angle_rad) {
    uint32_t da, db, dc;
    float sin_theta, cos_theta;
    pre_calc_sin_cos(angle_rad, &sin_theta, &cos_theta);
    inverse_park_transform(vd_ref, vq_ref, sin_theta, cos_theta, &hfoc->v_alpha, &hfoc->v_beta);
    svpwm(hfoc->v_alpha, hfoc->v_beta, hfoc->v_bus, hfoc->drv8353.pwm_resolution, &da, &db, &dc);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, da);//0-4499 level@10kHz
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, db);//0-4499 level@10kHz
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dc);//0-4499 level@10kHz
   // uint8_t len_str=sprintf(sendbuffer,"%.2f, %lu, %lu, %lu, %.2f, %.2f\r\n",angle_rad,da,db,dc, hfoc->v_alpha, hfoc->v_beta);
   //  HAL_UART_Transmit(&huart2, sendbuffer, len_str, 1000);
}


void DRV8301_CS_Select(void) {

	HAL_GPIO_WritePin(CS_SPI2_GPIO_Port, CS_SPI2_Pin, GPIO_PIN_RESET);

}

void DRV8301_CS_Deselect(void) {

	HAL_GPIO_WritePin(CS_SPI2_GPIO_Port, CS_SPI2_Pin, GPIO_PIN_SET);

}

void spi_write_DRV8301(uint8_t  reg, uint16_t  value)
{
	uint16_t controlWord = 0;
	uint16_t dummyRx = 0;
	// R/W bit (0 = Write) + Address (4 bits) + Data (11 bits)
	controlWord = (0 << 15) | ((reg & 0x0F) << 11) | (value & 0x07FF);
	DRV8301_CS_Select();
    HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&controlWord, (uint8_t*)&dummyRx, 1, HAL_MAX_DELAY);
    DRV8301_CS_Deselect();


}


uint16_t spi_read_DRV8301(uint8_t  reg)
{

	uint16_t controlWord = 0;
	uint16_t receivedData = 0;

	controlWord = (1 << 15) | ((reg & 0x0F) << 11);
	DRV8301_CS_Select();
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&controlWord, (uint8_t*)&receivedData, 1, HAL_MAX_DELAY);
	DRV8301_CS_Deselect();

	controlWord = (1 << 15) | (0x00 << 11);
	DRV8301_CS_Select();
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&controlWord, (uint8_t*)&receivedData, 1, HAL_MAX_DELAY);
	DRV8301_CS_Deselect();


	 uint8_t len_str=sprintf(sendbuffer,"%X\r\n",receivedData & 0x07FF);
	 HAL_UART_Transmit(&huart2, sendbuffer, len_str, 1000);
	 return (receivedData & 0x07FF);

}







int8_t foc_init(foc_t *hfoc, I2C_HandleTypeDef *hi2c)
{
  // hfoc->m_angle_offset =get_degree_as5600(hi2c,1.0);
   hfoc->pole_pairs=POLE_PAIR;
   hfoc->v_bus= 12.0;// 12V
   hfoc->drv8353.pwm_resolution=2249;

   hfoc->id_ctrl.e_deadband=0.5f;
   hfoc->id_ctrl.kp=30.0f;
   hfoc->id_ctrl.ki=10.25f;
   hfoc->iq_ctrl.e_deadband=0.5f;
   hfoc->iq_ctrl.kp=30.0f;
   hfoc->iq_ctrl.ki=10.25f;

   return 0;
    
}

int8_t foc_sensor_cal_elec_angle(foc_t *hfoc, I2C_HandleTypeDef *hi2c)
{

  float angle_deg = 0, diff;
  angle_deg=get_degree_as5600(hi2c,1.0);
  hfoc->m_angle_deg=angle_deg;
  // Normalize mechanical angle
  hfoc->m_angle_rad = DEG_TO_RAD(hfoc->m_angle_deg) - hfoc->m_angle_offset;

  // Nomalize   mechanical angle  between 0 <= theta < 2pi
   norm_angle_rad(&hfoc->m_angle_rad);

  
  // Calculate raw electric angle
  float e_rad = hfoc->m_angle_rad * hfoc->pole_pairs;

  //uint8_t len_str=sprintf(sendbuffer,">>  %.3f, %.3f\r\n",  hfoc->m_angle_deg, RAD_TO_DEG(hfoc->m_angle_rad ));
  	                                                                                    
 //HAL_UART_Transmit(&huart2, sendbuffer, len_str, 1000);

  
  // Handle sensor direction
  hfoc->sensor_dir=0;
   if (hfoc->sensor_dir == REVERSE_DIR) {
	  e_rad = TWO_PI - e_rad;
   }
 
    norm_angle_rad(&e_rad);
    hfoc->e_angle_rad=e_rad;
  return 0;
}

void myfoc_cal_encoder_misalignment(foc_t *hfoc) {

 open_loop_voltage_control(hfoc, VD_CAL, VQ_CAL, 0.0f);
 HAL_Delay(700);
 hfoc->e_angle_offset =0.0;
 float rad_offset = 0.0f;
 for (int i = 0; i < CAL_ITERATION; i++) {
    rad_offset += deg_to_rad(get_degree_as5600(&hi2c2,1.0));
   HAL_Delay(1);
  }
  open_loop_voltage_control(hfoc, 0.0f, 0.0f, 0.0f);
  rad_offset = rad_offset / (float)CAL_ITERATION;
  hfoc->e_angle_offset = rad_offset;

 if(is_send==1){
	 is_send=0;
     uint8_t len_str=sprintf(sendbuffer,">> e_angle_offset : %.3f\r\n", hfoc->e_angle_offset);
     HAL_UART_Transmit_DMA(&huart2, sendbuffer, len_str);
  }
}




void InitMagnetSensorState(foc_t *hfoc, I2C_HandleTypeDef *hi2c)
{
	int16_t rawAngle= get_raw_as5600(hi2c);
	float initial_angle_rad= (float)( (rawAngle *(2.0f*(float)M_PI)) /4095.0f);

	hfoc->as5600.initial_angle_rad=initial_angle_rad;
	hfoc->as5600.prev_angle_rad = initial_angle_rad;
	hfoc->as5600.full_rotations=0.0f;
	hfoc->as5600.now_angle=initial_angle_rad;
	hfoc->as5600.angle_el=0.0f;
	hfoc->as5600.prev_angle_filtered = initial_angle_rad; //speed loop only


	/*if(is_send==1){
		is_send=0;
	     uint8_t len_str=sprintf(sendbuffer,">> rawAngle : %.3f, %.3f\r\n",  RAD_TO_DEG(initial_angle_rad));
		 HAL_UART_Transmit_DMA(&huart2, sendbuffer, len_str);
	}*/


}

void Update_Magnet_Angle(foc_t *hfoc, I2C_HandleTypeDef *hi2c)
{

	int16_t rawAngle=get_raw_as5600(hi2c);
    float angle_rad=(float)( (rawAngle *(2.0f*(float)M_PI)) /4095.0f);

	//angle_rad =  hsmo.e_theta;
    float delta_angle_rad= angle_rad - hfoc->as5600.prev_angle_rad;

    //find absolute value float point number
   if (fabsf(delta_angle_rad) > 0.8f * 2.0f * (float)M_PI) {
          hfoc->as5600.full_rotations += (delta_angle_rad > 0.0f) ? -1 : 1;
    }
   hfoc->as5600.prev_angle_rad= angle_rad;


   hfoc->as5600.angle_el = _normalizeAngle((float)((SENSOR_DIRECTION * POLE_PAIR) *angle_rad) - hfoc->as5600.zero_electric_angle);
  hfoc->as5600.angle_el *=1.0f;
//   hfoc->as5600.angle_el= hsmo.e_theta; //test smo
   hfoc->as5600.now_angle = ((2.0f * (float)PI * hfoc->as5600.full_rotations) + (angle_rad));
 

   hfoc->as5600.angle_filtered =  (hfoc->as5600.rpm_alpha*hfoc->as5600.prev_angle_filtered) + (1.0f- hfoc->as5600.rpm_alpha)*hfoc->as5600.now_angle;
   hfoc->as5600.prev_angle_filtered = hfoc->as5600.angle_filtered;
   hfoc->actual_angle_rad = hfoc->as5600.now_angle;
  // printf("mAngle: %.3f, eAngle: %.3f\r\n", RAD_TO_DEG(hfoc->as5600.now_angle), RAD_TO_DEG(hfoc->as5600.angle_el));

}


void CalibrateZeroElectricAngle(foc_t *hfoc, I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim)
{

	hfoc->vq=  VOLTAGE_ALIGN;
	hfoc->vd = 0;
    setPhaseVoltage(hfoc, _3PI_2, &htim1);
    HAL_Delay(1000);

    hfoc->as5600.zero_electric_angle = 0.0f;
    Update_Magnet_Angle(hfoc, hi2c);

    hfoc->as5600.zero_electric_angle = hfoc->as5600.angle_el;
    HAL_Delay(50);
   // printf(">> zero_e_angle : %.3f\r\n ",hfoc->as5600.zero_electric_angle);

    hfoc->vq = 0;
    hfoc->vd = 0;
    setPhaseVoltage(hfoc, _3PI_2, &htim1);
    HAL_Delay(200);
}

float Convert_ADC_to_Current(uint32_t adc_value)
{
    float voltage = (((float)adc_value / ADC_RESOLUTION) * ADC_REF_VOLTAGE) - (ADC_REF_VOLTAGE / 2);
    float current = voltage / (SHUNT_RESISTANCE * CURRENT_AMP_GAIN);
    return current;
}

float ADC_to_I(int32_t adc_diff)
{

    float voltage = (((float)adc_diff / ADC_RESOLUTION) * ADC_REF_VOLTAGE);
    return voltage / (SHUNT_RESISTANCE * CURRENT_AMP_GAIN);
}


void getDQCurrents(foc_t *hfoc, float angle_el)
{
  //Clark
  float mid = 0.0;//(1.f/3.0)*(hfoc->drv8353.a + hfoc->drv8353.b + hfoc->drv8353.c);
  float ia_phase = hfoc->drv8353.a - mid;
  float ib_phase = hfoc->drv8353.b - mid;
  hfoc->i_alpha = ia_phase;
  hfoc->i_beta  = (ONE_BY_SQRT3 * ia_phase) + (TWO_BY_SQRT3 * ib_phase);
  //Park
 // float st = sinf(angle_el);
  //float ct = cosf(angle_el);
  float st = fast_sin(angle_el);
  float ct = fast_cos(angle_el);
  hfoc->id=  (hfoc->i_alpha * ct) + ( hfoc->i_beta * st);
  hfoc->iq=  (hfoc->i_beta * ct) - ( hfoc->i_alpha * st);

}

void PI_current_dq_debug(foc_t *hfoc)
{


	float error_d = hfoc->id_ref -  hfoc->drv8353.lowpass_d;
	float error_q = hfoc->iq_ref -  hfoc->drv8353.lowpass_q;

	float pTerm_d = hfoc->id_ctrl.kp * error_d;
	float pTerm_q = hfoc->iq_ctrl.kp * error_q;

	hfoc->id_ctrl.integral += hfoc->id_ctrl.ki*error_d * DT;
	hfoc->iq_ctrl.integral += hfoc->iq_ctrl.ki*error_q * DT;

	
	//printf("err_d = %.3f, err_q= %.3f || Id=  %.6f, Iq=%.6f\r\n", error_d, error_q,hfoc->id_ctrl.integral,hfoc->iq_ctrl.integral);

	hfoc->id_ctrl.integral  = _constrain(hfoc->id_ctrl.integral,
									   -INTEGRAL_D_MAX, INTEGRAL_D_MAX);
	hfoc->iq_ctrl.integral = _constrain(hfoc->iq_ctrl.integral,
									   -INTEGRAL_Q_MAX, INTEGRAL_Q_MAX);
	
	float iTerm_d = hfoc->id_ctrl.integral;
	float iTerm_q = hfoc->iq_ctrl.integral;

	hfoc->vd= pTerm_d + iTerm_d;
	hfoc->vq = pTerm_q + iTerm_q;

	const float U_MAX2 = VOLTAGE_LIMIT * VOLTAGE_LIMIT;
	float mag2 = (hfoc->vd * hfoc->vd)  + (hfoc->vq * hfoc->vq);

	if (mag2 > U_MAX2) {
	    float k = VOLTAGE_LIMIT / sqrtf(mag2);
	    hfoc->vd *= k;
	    hfoc->vq *= k;

	    hfoc->id_ctrl.integral -= error_d * DT * 0.5f;
	    hfoc->iq_ctrl.integral -= error_q * DT * 0.5f;


	    hfoc->id_ctrl.integral = _constrain(hfoc->id_ctrl.integral,
	                                       -INTEGRAL_D_MAX, INTEGRAL_D_MAX);
	    hfoc->iq_ctrl.integral = _constrain(hfoc->iq_ctrl.integral,
	                                       -INTEGRAL_Q_MAX, INTEGRAL_Q_MAX);
	}
}


void PI_current_dq(foc_t *hfoc)
{


	float error_d = hfoc->id_ref -  hfoc->drv8353.lowpass_d;
	float error_q = hfoc->iq_ref -  hfoc->drv8353.lowpass_q;

	float pTerm_d = hfoc->id_ctrl.kp * error_d;
	float pTerm_q = hfoc->iq_ctrl.kp * error_q;

	hfoc->id_ctrl.integral += error_d * DT;
	hfoc->iq_ctrl.integral += error_q * DT;

	hfoc->id_ctrl.integral  = _constrain(hfoc->id_ctrl.integral,
									   -INTEGRAL_D_MAX, INTEGRAL_D_MAX);
	hfoc->iq_ctrl.integral = _constrain(hfoc->iq_ctrl.integral,
									   -INTEGRAL_Q_MAX, INTEGRAL_Q_MAX);

	
	float iTerm_d = hfoc->id_ctrl.ki*hfoc->id_ctrl.integral;
	float iTerm_q = hfoc->iq_ctrl.ki*hfoc->iq_ctrl.integral;

	hfoc->vd= pTerm_d + iTerm_d;
	hfoc->vq = pTerm_q + iTerm_q;
//	printf("err_d = %.3f, err_q= %.3f || Id=  %.6f, %.3f <> Iq=%.6f, %.3f\r\n", 
	//	error_d, error_q,hfoc->id_ctrl.integral,hfoc->vd,hfoc->iq_ctrl.integral,hfoc->vq);

	const float U_MAX2 = VOLTAGE_LIMIT * VOLTAGE_LIMIT;
	float mag2 = (hfoc->vd * hfoc->vd)  + (hfoc->vq * hfoc->vq);

	if (mag2 > U_MAX2) {
	    float k = VOLTAGE_LIMIT / sqrtf(mag2);
	    hfoc->vd *= k;
	    hfoc->vq *= k;

	    hfoc->id_ctrl.integral -= error_d * DT * 0.5f;
	    hfoc->iq_ctrl.integral -= error_q * DT * 0.5f;


	    hfoc->id_ctrl.integral = _constrain(hfoc->id_ctrl.integral,
	                                       -INTEGRAL_D_MAX, INTEGRAL_D_MAX);
	    hfoc->iq_ctrl.integral = _constrain(hfoc->iq_ctrl.integral,
	                                       -INTEGRAL_Q_MAX, INTEGRAL_Q_MAX);
	}
}

void LPF_current_dq(foc_t *hfoc)
{
	if(!hfoc->drv8353.initialized){
		hfoc->drv8353.prev_d =hfoc->id;
		hfoc->drv8353.prev_q = hfoc->iq;
		hfoc->drv8353.initialized = 1;
	}
	// hfoc->drv8353.lowpass_alpha_d =1  will fillter nois is low
	hfoc->drv8353.lowpass_d = hfoc->drv8353.prev_d + hfoc->drv8353.lowpass_alpha_d * (hfoc->id  - hfoc->drv8353.prev_d);
	hfoc->drv8353.lowpass_q = hfoc->drv8353.prev_q + hfoc->drv8353.lowpass_alpha_q * (hfoc->iq  - hfoc->drv8353.prev_q);
	hfoc->drv8353.prev_d = hfoc->drv8353.lowpass_d;
	hfoc->drv8353.prev_q = hfoc->drv8353.lowpass_q;

	//printf("lfp_d=%.3f, lpf_q=%.3f\r\n", hfoc->drv8353.lowpass_d,hfoc->drv8353.lowpass_q);
}

//SVPWM
void setPhaseVoltage(foc_t *hfoc, float angle_el, TIM_HandleTypeDef *htim)
{
   // float _sa = sinf(angle_el);
    //float _ca = cosf(angle_el);
	float _sa = fast_sin(angle_el);
	float _ca = fast_cos(angle_el);

    hfoc->v_alpha =  _ca*hfoc->vd  -  _sa*hfoc->vq;
    hfoc->v_beta  =  _sa*hfoc->vd  +  _ca*hfoc->vq;

    //int8_t smo_ret = smo_update_arctan(&hsmo, hfoc->v_alpha, hfoc->v_beta, hfoc->i_alpha, hfoc->i_beta);
	
    hfoc->va = hfoc->v_alpha;
    hfoc->vb = -0.5*hfoc->v_alpha  +   SQRT3_BY_TWO*hfoc->v_beta;
    hfoc->vc = -0.5*hfoc->v_alpha  -   SQRT3_BY_TWO*hfoc->v_beta;

    float center = VOLTAGE_LIMIT / 2.0f;
    float Umin = fminf(hfoc->va, fminf(hfoc->vb,  hfoc->vc));
    float Umax = fmaxf(hfoc->va, fmaxf(hfoc->vb,  hfoc->vc));

    center -= (Umax + Umin) / 2.0f;

    hfoc->va  += center;
    hfoc->vb  += center;
    hfoc->vc  += center;

    hfoc->va = _constrain(hfoc->va, 0.0f, VOLTAGE_LIMIT);
    hfoc->vb = _constrain(hfoc->vb, 0.0f, VOLTAGE_LIMIT);
    hfoc->vc = _constrain(hfoc->vc, 0.0f, VOLTAGE_LIMIT);

    hfoc->da= _constrain(hfoc->va/ VOLTAGE_POWER_SUPPLY, 0.0f, 1.0f);
    hfoc->db = _constrain(hfoc->vb / VOLTAGE_POWER_SUPPLY, 0.0f, 1.0f);
    hfoc->dc = _constrain(hfoc->vc / VOLTAGE_POWER_SUPPLY, 0.0f, 1.0f);

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t)(hfoc->da * period));
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)(hfoc->db * period));
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, (uint32_t)(hfoc->dc * period));

}


int8_t FOC_Torque_Control(void)
{
	int8_t ret=0;
	static uint16_t event_speed_loop_count = 0;

	Update_Magnet_Angle(&hfoc, &hi2c2);
		
    if(hfoc.foc_mode == FOC_MODE_SENSORED)
    {

	    event_speed_loop_count++;
	 
    	if(event_speed_loop_count >= SPEED_CONTROL_CYCLE) //Loop 1khz from PWM=20kH
    	{
    	      hfoc.actual_rpm= get_rpm_as5600(&hfoc);
		 
    	 event_speed_loop_count=0;
    	 ret=1;
    	}
    	
    }

    getDQCurrents(&hfoc, hfoc.as5600.angle_el);
    LPF_current_dq(&hfoc);
    PI_current_dq_debug(&hfoc);
    setPhaseVoltage(&hfoc,hfoc.as5600.angle_el, &htim1);

    return ret;
}

 void FOC_FastLoop(void)
 {
	 switch(hfoc.control_mode)
	 {
	  case TORQUE_CONTROL_MODE:  FOC_Torque_Control();break;
	  case SPEED_CONTROL_MODE: speed_control(sp_input);break;
	  case POSITION_CONTROL_MODE:
		  if(FOC_Torque_Control()==1)
		  {
			  foc_position_control_update(&hfoc, 36000.0f);
		  }
		  break;
	  case  CALIBRATION_MODE  :cal_pid_relay();	 break;
	  case  CALIBRATION_RS_L_MODE: meas_inj_dq_process(&hfoc, FOC_TS) ;break;
		    
	  case AUDIO_MODE:
	       //audio_loop(&hfoc, &htim1);
		   audio_test(&hfoc); break;
	      
	  case IDEL_MODE:break;
	  	    
	      
	
	 }


 }


 void FOC_OpenLoopSensorTest(void)
 {
	 Update_Magnet_Angle(&hfoc, &hi2c2);
	 getDQCurrents(&hfoc, hfoc.as5600.angle_el);
	 LPF_current_dq(&hfoc);

	 static float theta = 0.0f;


	  theta += TEST_W_RAD * DT;
	  if (theta > TWO_PI) theta -= TWO_PI;

	   hfoc.vd = 0.0f;
	   hfoc.vq  = TEST_UQ_V;
	   setPhaseVoltage(&hfoc,theta, &htim1);

 }

 float get_rpm_as5600(foc_t *hfoc)
 {
     
      float angle_diff = hfoc->as5600.angle_filtered - hfoc->as5600.prev_angle_rpm;
      hfoc->as5600.prev_angle_rpm= hfoc->as5600.angle_filtered;
	
	  //hfoc->actual_angle_rad

      // Calculate RPM
      float rpm_instant = (angle_diff * 60.0f) / (SPEED_TS * TWO_PI);

      // Very low RPM clamping (0.1 RPM resolution)
       if (fabsf(rpm_instant) < 0.1f) {
        	rpm_instant = 0.0f;
        }
     else if(fabsf(rpm_instant) > 900.0f && lock_speed_start==0)
	  {
	    	rpm_instant = 0.0f;
			lock_speed_start=1;
	   }

      return rpm_instant;


 }


 void foc_speed_control_update(foc_t *hfoc, float _rpm_error) {

 	if (hfoc == NULL || (hfoc->control_mode != SPEED_CONTROL_MODE && hfoc->control_mode != POSITION_CONTROL_MODE)) {
 		hfoc->speed_ctrl.integral = 0.0f;
 		hfoc->speed_ctrl.last_error = 0.0f;
 		return;
 	}

 	hfoc->iq_ref = pi_control(&hfoc->speed_ctrl, _rpm_error);
	
		
 }


 void foc_position_control_update(foc_t *hfoc, float deg_reference) {

     if (hfoc == NULL || hfoc->control_mode != POSITION_CONTROL_MODE) {
         hfoc->pos_ctrl.integral = 0.0f;
         hfoc->pos_ctrl.last_error = 0.0f;
         return;
     }

     // Normalize reference angle to 0-360 degrees
    // deg_reference = fmodf(deg_reference, TWO_PI);
    // if (deg_reference < 0) {
    //     deg_reference += TWO_PI;
    // }

     if (hfoc->loop_count >= (POSITION_CONTROL_CYCLE / SPEED_CONTROL_CYCLE)) {
         hfoc->loop_count = 0;

         // Calculate shortest path error with wrap-around

      
	     static uint32_t cnt_ramp_up_sp=0;
	     static float sum_delta=0.0;

	     sp_pos =SCurve_Update(&motion,POSITION_TS);

		/*
		if(end_pos==0)
		{
         cnt_ramp_up_sp++;
		if(cnt_ramp_up_sp < cnt_ramp)
			{
			   sum_delta +=((step_cruise-step_ramp)/cnt_ramp);
			 if(deg_reference>0.0)
		
	    	  sp_pos= sp_pos+step_ramp+sum_delta;
			 else
			 	sp_pos= sp_pos-step_ramp- sum_delta;
			
			}
		
		else if(cnt_ramp_up_sp>=cnt_ramp && cnt_ramp_up_sp < (cnt_ramp+cnt_cuise))
			{

		
				 if(deg_reference>0.0)	
	              sp_pos = sp_pos+step_cruise;
				 else
				 	sp_pos = sp_pos-step_cruise;
			   
			 if(fabsf(sp_pos)>= deg_reference)
			 	 sp_pos=deg_reference;

			 	
			}
		else if(cnt_ramp_up_sp >= cnt_ramp+cnt_cuise)
				{
				    sum_delta=0.0;
				    cnt_ramp_up_sp =0;
                    end_pos=1;
					sp_pos=deg_reference;
				}
		
		 }
		
         */
        /*
		 if(end_pos==0)
		{
         cnt_ramp_up_sp++;
		if(cnt_ramp_up_sp <  cnt_ramp)
			{
			 if(deg_reference>0.0)
	    	  sp_pos= sp_pos+step_ramp;
			 else
			 	sp_pos= sp_pos-step_ramp;
			}
	    else if(cnt_ramp_up_sp >  cnt_ramp)
				{
				    cnt_ramp_up_sp =0;
                    end_pos=1;
					sp_pos=deg_reference;
				}
		
		 }*/
         
	 
         float error = (DEG_TO_RAD(sp_pos)+hfoc->as5600.initial_angle_rad) - hfoc->actual_angle_rad;
	         _pos_error =error;

         // Handle wrap-around for shortest path
        /* if (error > PI) {
             error -= TWO_PI;
         } else if (error < -PI) {
             error += TWO_PI;
         }*/

         hfoc->rpm_ref = pid_control(&hfoc->pos_ctrl, error);
 	
     }

	 

     hfoc->loop_count++;
	 


     foc_speed_control_update(hfoc, hfoc->rpm_ref);

	 //  if(is_send==1)
	 // {
	  /*
	    present_pos=hfoc->actual_angle_rad;
	     if (present_pos > PI) {
             present_pos -= TWO_PI;
         } else if (present_pos < -PI) {
            present_pos += TWO_PI;
         }*/
	 
	 // is_send=0;
	//  }
 }






 uint16_t audio_test(foc_t *hfoc)
 {
     static float angle = 0.0f;
     static uint32_t sample_count = 0;
     static uint16_t cnt_loop_audio=0;
     const float fs = 20000.0f;       // audio loop = 20 kHz
     const float freq = 500.0f;       // เสียง 500 Hz
     const float voltage = 12.0f;

     /*
      * =====================================================
      * Beep timing
      * =====================================================
      *
      * 200 ms ON
      * 100 ms OFF
      *
      * ได้เสียงประมาณ:
      *
      * ตึด   ตึด   ตึด   ตึด
      */
     const uint32_t beep_on  = 200;   // ms
     const uint32_t beep_off = 100;   // ms

     const uint32_t period_ms =
         beep_on + beep_off;

     /*
      * Current time in milliseconds
      *
      * ใช้ sample_count �?ทน HAL_GetTick()
      */
     uint32_t samples_per_period =
         (uint32_t)((period_ms / 1000.0f) * fs);

     uint32_t samples_on =
         (uint32_t)((beep_on / 1000.0f) * fs);


     /*
      * =====================================================
      * Determine ON / OFF
      * =====================================================
      */

     uint32_t position =
         sample_count % samples_per_period;

     float amplitude;

     if (position < samples_on)
     {
         amplitude = voltage;
     }
     else
     {
         amplitude = 0.0f;
     }


     /*
      * =====================================================
      * Generate electrical angle
      * =====================================================
      */

     angle += TWO_PI * freq / fs;

     if (angle >= TWO_PI)
     	{

         angle -= TWO_PI;
     	}


     /*
      * =====================================================
      * FOC voltage
      * =====================================================
      */

     float vd = amplitude;
     float vq = 0.0f;


     hfoc->vd = vd;
     hfoc->vq = vq;


     open_loop_voltage_control(
         hfoc,
         vd,
         vq,
         angle
     );


     /*
      * =====================================================
      * Sample counter
      * =====================================================
      */

     sample_count++;

     if (sample_count >= (samples_per_period*3))
     {
         sample_count = 0;
		  hfoc->control_mode= POSITION_CONTROL_MODE;

     }
     return 0;
 }




void meas_inj_dq_process(foc_t *hfoc, float ts) {

    const uint32_t wt = 256; // waiting time

    if (hfoc->meas_inj_start_flag) {
        float vd = 0.0f, vq = 0.0f;
        if (hfoc->meas_inj_target == RS) {
            vd = hfoc->meas_inj_amp;
            vq = 0.0f;
        }
        else {
            float angle = hfoc->meas_inj_omega * hfoc->meas_inj_n * ts;
            float v_inj = hfoc->meas_inj_amp * fast_sin(angle);
            if (hfoc->meas_inj_target == LD) {
                vd = v_inj;
                vq = 0.0f;
            }
            else if (hfoc->meas_inj_target == LQ) {
                vd = 0.0f;
                vq = v_inj;
            }
        }

        // float theta_e = hfoc->e_angle_rad_comp;
        float theta_e = 0.0f;

        open_loop_voltage_control(hfoc, vd, vq, theta_e);

    
        float sin_theta, cos_theta;
        float id, iq;

		float mid = 0.0;//(1.f/3.0)*(hfoc->drv8353.a + hfoc->drv8353.b + hfoc->drv8353.c);
        float ia_phase = hfoc->drv8353.a - mid;
        float ib_phase = hfoc->drv8353.b - mid;
        pre_calc_sin_cos(theta_e, &sin_theta, &cos_theta);
        clarke_park_transform( ia_phase, ib_phase, sin_theta, cos_theta, &id, &iq);

        if (hfoc->meas_inj_n >= wt) {
		//	printf(">>%d\r\n",hfoc->meas_inj_n);
            if (hfoc->meas_inj_target == RS || hfoc->meas_inj_target == LD) {
                Vd_buff[hfoc->meas_inj_n - wt] = vd;
                Id_buff[hfoc->meas_inj_n - wt] = id;
            }
            else if (hfoc->meas_inj_target == LQ) {
                Vq_buff[hfoc->meas_inj_n - wt] = vq;
                Iq_buff[hfoc->meas_inj_n - wt] = iq;
            }
        }

        hfoc->meas_inj_n++;
		
        if (hfoc->meas_inj_n >= (MAX_I_SAMPLE + wt)) {
            hfoc->meas_inj_n = 0;
            hfoc->meas_inj_start_flag = 0; 
            open_loop_voltage_control(hfoc, 0, 0, 0);
        }
    }
}


static void start_measure(inject_taregt_t target) {
  hfoc.meas_inj_target = target;
  HAL_Delay(1000);
  hfoc.meas_inj_start_flag = 1;
  // waiting process
  while(hfoc.meas_inj_start_flag) {
  HAL_Delay(1);
  }
}
static int measure_R(float vdc)
 {
  if (hfoc.v_bus  < 10.0f) {
    return -1;
  }

  hfoc.meas_inj_amp = vdc;

  memset(Vd_buff, 0, sizeof(Vd_buff));
  memset(Id_buff, 0, sizeof(Id_buff));

  start_measure(RS);
  estimate_resistance(&hfoc);

  //m_config.Rs = hfoc.Rs;
  float badwidth =50.0f;
  float Omega=2.0*PI*badwidth;
  float id_ki=hfoc.Rs*Omega;
  if(is_send==1)
  	  {
  	  uint8_t len_str=sprintf(sendbuffer,"Estimate Resistance @(V=%.2f, Rs= %f,  id_ki= %.3f)\r\n", hfoc.meas_inj_amp, hfoc.Rs,id_ki );
  	  HAL_UART_Transmit_DMA(&huart2, sendbuffer, len_str);
  	  is_send=0;
  	}



  return 0;
}


static int measure_L(float f, float amp) {
  if (hfoc.v_bus  < 10.0f) {
    return -1;
  }

  hfoc.meas_inj_freq = f;
  hfoc.meas_inj_amp = amp;
  hfoc.meas_inj_omega = TWO_PI * hfoc.meas_inj_freq;

  memset(Vd_buff, 0, sizeof(Vd_buff));
  memset(Id_buff, 0, sizeof(Id_buff));
  memset(Vq_buff, 0, sizeof(Vq_buff));
  memset(Iq_buff, 0, sizeof(Iq_buff));

  start_measure(LD);
  start_measure(LQ);
  estimate_inductance(&hfoc, FOC_TS);
 // m_config.Ld = hfoc.Ld;
  //m_config.Lq = hfoc.Lq;

  for (int i = 0; i < MAX_I_SAMPLE; i++) {
    float buffer_val[4] = {
      Vd_buff[i], Vq_buff[i],
      Id_buff[i], Iq_buff[i]
    };

  }

  float badwidth =50.0f;
  float Omega=2.0*PI*badwidth;
  float id_kp=hfoc.Ld*Omega;
  float iq_kp=hfoc.Lq*Omega;

  if(is_send==1)
   	  {
   	  uint8_t len_str=sprintf(sendbuffer,"Estimate Inductance @(f=%.2fHz), Ld= %f,  Lq=%f, Rs=%.6f, id_kp=%.3f, iq_kp=%.3f)\r\n",
   			hfoc.meas_inj_freq, hfoc.Ld, hfoc.Lq, hfoc.Rs, id_kp,iq_kp);
   	  HAL_UART_Transmit_DMA(&huart2, sendbuffer, len_str);
   	  is_send=0;
   	}

  return 0;
}


void estimate_resistance(foc_t *hfoc) {
    float mean_vd = 0, mean_id = 0;

    for (int i = 0; i < MAX_I_SAMPLE; i++) {
        mean_vd += Vd_buff[i];
        mean_id += Id_buff[i];
	  //  printf("%.6f\r\n",Id_buff[i]);

    }
   // printf(" Vd =%.6f, id= %.6f\r\n",mean_vd, mean_id);
    mean_vd /= MAX_I_SAMPLE;
    mean_id /= MAX_I_SAMPLE;


    hfoc->Rs = mean_vd / mean_id;

}

void estimate_inductance(foc_t *hfoc, float ts) {
    float Vc_d = 0, Vs_d = 0, Ic_d = 0, Is_d = 0;
    float Vc_q = 0, Vs_q = 0, Ic_q = 0, Is_q = 0;
    float mean_vd = 0, mean_vq = 0, mean_id = 0, mean_iq = 0;

    // Remove DC offset
    for (int i = 0; i < MAX_I_SAMPLE; i++) {
        mean_vd += Vd_buff[i];
        mean_vq += Vq_buff[i];
        mean_id += Id_buff[i];
        mean_iq += Iq_buff[i];
    }
    mean_vd /= MAX_I_SAMPLE;
    mean_vq /= MAX_I_SAMPLE;
    mean_id /= MAX_I_SAMPLE;
    mean_iq /= MAX_I_SAMPLE;

    // Single-frequency DFT
    for (int i = 0; i < MAX_I_SAMPLE; i++) {
        float angle = hfoc->meas_inj_omega * i * ts;

        float vd = Vd_buff[i] - mean_vd;
        float vq = Vq_buff[i] - mean_vq;
        float id = Id_buff[i] - mean_id;
        float iq = Iq_buff[i] - mean_iq;

        Vc_d += vd * fast_cos(angle);
        Vs_d += vd * fast_sin(angle);
        Ic_d += id * fast_cos(angle);
        Is_d += id * fast_sin(angle);

        Vc_q += vq * fast_cos(angle);
        Vs_q += vq * fast_sin(angle);
        Ic_q += iq * fast_cos(angle);
        Is_q += iq * fast_sin(angle);
    }

    float norm = 2.0f / MAX_I_SAMPLE;
    Vc_d *= norm; Vs_d *= norm;
    Ic_d *= norm; Is_d *= norm;
    Vc_q *= norm; Vs_q *= norm;
    Ic_q *= norm; Is_q *= norm;

    // V & I amplitude
    float Vd_mag = sqrtf(Vc_d * Vc_d + Vs_d * Vs_d);
    float Id_mag = sqrtf(Ic_d * Ic_d + Is_d * Is_d);
    float Vq_mag = sqrtf(Vc_q * Vc_q + Vs_q * Vs_q);
    float Iq_mag = sqrtf(Ic_q * Ic_q + Is_q * Is_q);

    // (phi = arctan(Vs/Vc) - arctan(Is/Ic))
    float phi_d = atan2f(Vs_d, Vc_d) - atan2f(Is_d, Ic_d);
    float phi_q = atan2f(Vs_q, Vc_q) - atan2f(Is_q, Ic_q);

    // Impedansi & parameters
    float Zd_mag = Vd_mag / Id_mag;
    float Zq_mag = Vq_mag / Iq_mag;

   // float Rs_d = Zd_mag * cosf(phi_d); //test
   // float Rs_q = Zq_mag * cosf(phi_q); //test
    float Ld_est = (Zd_mag * sinf(phi_d)) / hfoc->meas_inj_omega;
    float Lq_est = (Zq_mag * sinf(phi_q)) / hfoc->meas_inj_omega;

  // hfoc->Rs = (Rs_d + Rs_q) * 0.5; //test
    hfoc->Ld = fabs(Ld_est);
    hfoc->Lq = fabs(Lq_est);
}

static void calibration_seq(void) {
  
 measure_R(0.1f);
 measure_L(400.0f, 1.25f);


}

int8_t relay_autotune(foc_t *hfoc, float delta_ts, float input, uint8_t type_pid)
{

	static uint16_t cnt_pu=0;
	static uint16_t start_cnt_pu=0;
	uint16_t index_T1=0, index_T2=0;
	float Pu=0.0f;
	float avr_amp=0.0f;
	float Kcu=0.0f,Kc=0.0f, tor_i=0.0f, tor_d=0.0f;
	float h=0.0f,A=0.0f;

    h= fabsf( input);

	for(uint16_t i=0;i < cnt_relay-1 ;i++ )
		{


                if( (Iq_buff_relay[i-1] > Iq_buff_relay[i-2])  && (Iq_buff_relay[i-1] >= Iq_buff_relay[i]))
                    {

                    	 index_T1 = i-1;
                    }
                 else  if( (Iq_buff_relay[i-1] < Iq_buff_relay[i-2])  && (Iq_buff_relay[i-1] <= Iq_buff_relay[i]))
                   {
                	     index_T2 = i-1;
                   }


	}

	for(uint16_t i=0;i < cnt_relay-1 ;i++ )
	{


	                 if( (Iq_buff_relay[i-1] > Iq_buff_relay[i-2])  && (Iq_buff_relay[i-1] >= Iq_buff_relay[i]))
	                    {
	                	    if( i > 100)
	                	    {
								 start_cnt_pu++;
								 if(start_cnt_pu==1) // find Peak 1t  amp.
									cnt_pu++;
								 else if(start_cnt_pu==2)// find Peak 2t amp.
								 {
									cnt_pu++;
									start_cnt_pu=0;
									break;

								 }
	                	    }


	                    }

	                 if(start_cnt_pu==1)
	                    cnt_pu++;


	}
	printf("PEAK MAX: INDEX_T1=%d , %.6f\r\n",
		index_T1, Iq_buff_relay[index_T1] );
	printf("PEAK MIN: INDEX_T2=%d  , %.6f\r\n",
			 index_T2, Iq_buff_relay[index_T2]);
    avr_amp = (fabsf(Iq_buff_relay[index_T1]) + fabs(Iq_buff_relay[index_T2] ) )*0.5f;
	printf("AVR Amp(a): %.6f\r\n", avr_amp );





	  //Tyreus-Luyben Method
	  A= avr_amp ;
	  Kcu= (4.0*h)/(_PI*A);
	  Pu=(float)cnt_pu*delta_ts;
	  printf("Pu: %.6f , A=%.6f\r\n",Pu,A);

	  if(type_pid == TORQUE_CONTROL_MODE) // PI control only
	  	{
	  	  Kc= 0.3125*Kcu;
		  tor_i= 2.2*Pu;
		  hfoc->iq_ctrl.kp= Kc;
		  hfoc->iq_ctrl.ki= Kc/tor_i;
		  hfoc->id_ctrl.kp=  hfoc->iq_ctrl.kp;
		  hfoc->id_ctrl.ki= hfoc->iq_ctrl.ki;
		  printf("TOR:kp= %.6f, Ki=%.6f\r\n", hfoc->iq_ctrl.kp, hfoc->iq_ctrl.ki);
		 // hfoc->iq_ref=0.1;
	  	}
	  
   else if(type_pid == SPEED_CONTROL_MODE) // PI control only
	    {
		    float bandwid_speed=1.0f; //10Hz
	        Kc= 0.3125*Kcu;
	  	    tor_i= 2.2*Pu;
			hfoc->speed_ctrl.kp= Kc/bandwid_speed;
			hfoc->speed_ctrl.ki= Kc/tor_i;
			
			printf("SPEED:kp= %.6f, Ki=%.6f\r\n", hfoc->speed_ctrl.kp,hfoc->speed_ctrl.ki);
			 
		}
   else if(type_pid ==  POSITION_CONTROL_MODE) // PID controller
   	{

            Kc= 0.4545*Kcu;
	  	    tor_i= 2.2*Pu;
			tor_d= 0.1587*Pu;
			hfoc->pos_ctrl.kp= Kc;
			hfoc->pos_ctrl.ki= Kc/tor_i;
			hfoc->pos_ctrl.kd = Kc*tor_d;
			
			printf("POSITION:kp= %.6f, Ki=%.6f, Kd =%.6f\r\n",hfoc->pos_ctrl.kp,hfoc->pos_ctrl.ki,hfoc->pos_ctrl.kd);


   	}

   cnt_pu=0;
   start_cnt_pu=0;
    memset(Iq_buff_relay,0 ,cnt_relay);
   //HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_ALL);  // Starts CH1
   HAL_GPIO_WritePin(GPIOB, EN_GATE_Pin, GPIO_PIN_RESET);
 //  HAL_Delay(2000);
 //  hfoc->control_mode= SPEED_CONTROL_MODE;
//   HAL_GPIO_WritePin(GPIOB, EN_GATE_Pin, GPIO_PIN_SET);
 //  printf("Start Tor Loop...\r\n");
  // HAL_Delay(2000);

  

	

}


uint8_t speed_control(float input_rpm)

{
	//const float acc_rpm = 1.0f;

	if( FOC_Torque_Control()==1)
			 {
	
				 
				  
				   
				  if (input_rpm > sp_rpm) {
					  // sp_rpm += acc_rpm;
					   sp_rpm +=SPEED_TS*input_rpm;
					   if (sp_rpm > input_rpm)
					   	  sp_rpm = input_rpm;
				   }
				   else if (input_rpm < sp_rpm) {
						//sp_rpm -= acc_rpm;
					    sp_rpm +=SPEED_TS*input_rpm;
						
						if (sp_rpm < input_rpm)
							sp_rpm = input_rpm;
				   }
				  
	               //  sp_rpm= input_rpm;
				   _rpm_error =  sp_rpm	- hfoc.actual_rpm;
				   foc_speed_control_update(&hfoc, _rpm_error);
				   return  1;
	
			 }

    return 0;
}


void cal_pid_relay(void)
{
    static float error_cal=0.0;
	static float deadband_speed=0.5;
	static float deadband_position=0.1;
	static float deadband_Iq=0.01;
	static float out_speed=0.0f;
	static uint16_t cnt_pos=0;
	
	if( start_cal == 0)
	{
	
	if(mode_cal_pid==CAL_TORQUE)
	{
    
	if(counter_relay_max < 20)
	{
	 counter_relay++;
	 if(counter_relay<=50)
		{
		
		   hfoc.iq_ref=0.5;
		
		}
	 else if(counter_relay>50 && counter_relay <=100)
		{
	   
		   hfoc.iq_ref=-0.5;
		}
	 else if(counter_relay>100)
		{
		   counter_relay=0;
		   counter_relay_max++;
		}
	 
	  
		FOC_Torque_Control();	
		Iq_buff_relay[cnt_relay]= hfoc.drv8353.lowpass_q;
			
			
		cnt_relay++;
		}
		 else
		 start_cal=1;
		 
		 
         /*
	     cnt_pos++;
	     if(cnt_pos<=50)
		 	hfoc.iq_ref=0.5;
		 else if(cnt_pos>50 && cnt_pos <=100)
		 {
		 	 hfoc.iq_ref=-0.5;
			
		 }
		 else if(cnt_pos> 100)
		 {
		 	cnt_pos=0;
		 }
		 FOC_Torque_Control();	
		   counter_relay_max++;
		 if(counter_relay_max>=20000) 
		 	{
                 counter_relay_max=20000;
				 Iq_buff_relay[cnt_relay]=  hfoc.drv8353.lowpass_q;
				cnt_relay++;
				if(cnt_relay >=2000)
						{
                         
						  counter_relay_max=0;
						  start_cal=1;
						}
				 
		 	}*/
	


		}
	 else if(mode_cal_pid==CAL_SPEED)
		{
		 if(FOC_Torque_Control()==1)
			{
			               
			  /* error_cal = sp_input-hfoc.actual_rpm;
			  if(error_cal > deadband_speed)
			  	hfoc.iq_ref=0.2;
			  else if(error_cal < -deadband_speed)
			  	hfoc.iq_ref=-0.2;
			  */
			    cnt_pos++;
			    if(cnt_pos ==1)
					hfoc.iq_ref=0.2;
				else if(cnt_pos ==2)
				{
				 hfoc.iq_ref=-0.2;
			      cnt_pos=0;
				}

			    
			 	  
			  counter_relay_max++;
			  if(counter_relay_max>=5000) 
			  	{
				counter_relay_max=5000;

                 	Iq_buff_relay[cnt_relay]= hfoc.actual_rpm;
					cnt_relay++;
					if(cnt_relay >=2000)
						{
                         
						  counter_relay_max=0;
						  start_cal=1;
						}
			  	}
	
			}
		
	
	
	
		}
	 else if(mode_cal_pid==CAL_POSITION)
		{

		      if( FOC_Torque_Control()==1)
		 	    {
                      
		 			
                     if (hfoc.loop_count >= (POSITION_CONTROL_CYCLE / SPEED_CONTROL_CYCLE)) {
                      hfoc.loop_count = 0;

					  cnt_pos++;
					  if(cnt_pos<=1000)
					   error_cal =  DEG_TO_RAD(sp_input)- hfoc.actual_angle_rad;
					  else if(cnt_pos>1000 && cnt_pos<=2000)
					  	{
					  	error_cal =  -DEG_TO_RAD(sp_input)- hfoc.actual_angle_rad;
							//cnt_pos=0;
					  	}
					  else if(cnt_pos>2000)
					  	cnt_pos=0;
					  
					   if(error_cal > deadband_position)
					  	out_speed=100.0;
					  else if(error_cal <-deadband_position)
					  	out_speed=-100.0;


					  /*
					   if(cnt_pos==100)
							out_speed=200.0;
					   else if(cnt_pos==200)
					   {
						   out_speed=-200.0;
						   cnt_pos=0;
					   }*/

					   speed_relay =  out_speed;
                      
					  counter_relay_max++;
					  if(counter_relay_max>=5000) 
			  	      {
				         counter_relay_max=5000;

	                 	Iq_buff_relay[cnt_relay]= hfoc.actual_angle_rad;
						cnt_relay++;
						if(cnt_relay >=2000)
							{
	                         
							   counter_relay_max=0;
							   start_cal=1;
							   hfoc.loop_count=0;
							   cnt_pos=0;
							}
				  	    }

					  
                     }
					 
					 hfoc.loop_count++;

					 _rpm_error =  out_speed- hfoc.actual_rpm;
					 hfoc.iq_ref = pi_control(&hfoc.speed_ctrl, _rpm_error);


		 	  }

		}
	 else if(mode_cal_pid== CAL_TORQUE_SPEED_POSITION)
	 	{
	 	   start_cal=1;
		   hfoc.loop_count=0;
		   cnt_pos=0;
	 	}
	
	
	 }


}


void cal_auto_torque_speed(void)
{

   if(cal_auto == 1) // Auto cal
   	{
   	   HAL_GPIO_WritePin(GPIOB, EN_GATE_Pin, GPIO_PIN_SET);
   	   hfoc.loop_count=0;
   	   cal_auto_step++;
	   switch(cal_auto_step)
	   	{
          case 1: mode_cal_pid=CAL_TORQUE; start_cal=0;    HAL_Delay(3000);  break;
		  case 2: mode_cal_pid=CAL_SPEED;  start_cal=0;    sp_input = 0.0f;  HAL_Delay(3000);    break;
		  case 3: mode_cal_pid= CAL_POSITION; start_cal=0;  sp_input = 2.5f;  HAL_Delay(3000); break;
		  case 4:  
		  	      start_cal =2; 
		          cal_auto_step=0; 
		          cal_auto=0;
		          mode_cal_pid=CAL_TORQUE_SPEED_POSITION;
				  hfoc.control_mode=IDEL_MODE;
				  HAL_GPIO_WritePin(GPIOB, EN_GATE_Pin, GPIO_PIN_RESET);
				  printf("CAL TOR+SPEED+POS Complete \r\n");
				  
		          break;
		  default: break;
          
	   	}


   	}
   else if(cal_auto==0)
   	  start_cal =2;

}


void  calculate_Trapezoidal_profile(
    float go_angle, float max_rpm_speed, float max_rpm_acc, float ts, 
    float *step_ramp, uint32_t *cnt_ramp,
    float *step_cruise, uint32_t *cnt_cuise)
{
  float ta,da, tud, total_dis,cruise_time, total_time;
  float tmp=0;
  float time_ramp_ms=0;
  
  
  ta =  max_rpm_speed/max_rpm_acc;
  da= 0.5*max_rpm_acc* ta*ta;
  if(da < go_angle)
  	{
        tud= da*1.0;
		total_dis=go_angle-tud;
		cruise_time=total_dis/max_rpm_speed;
		total_time =  ta+cruise_time+0.0;

		printf("Ta=%.6f, Cruise Time=%.6f   total=%.6f \r\n", ta, cruise_time, total_time);
		
        tmp=ta/ts;
		*cnt_ramp=(uint32_t)tmp;
		*step_ramp =da/tmp;
		tmp =  cruise_time/ts;
		*cnt_cuise=(uint32_t)tmp;
		*step_cruise=total_dis/tmp; 

		printf("STEP RAMP =%.6f, Count RAMP=%lu ,  STEP CRUISE =%.6f, Count Cruise =%lu \r\n",
			*step_ramp, *cnt_ramp, *step_cruise, *cnt_cuise);

		
  	}
  else
  	{
		   ta=   fabsf(go_angle/max_rpm_speed);
		  tmp = ta/ts;
		  *cnt_ramp=  (uint32_t)tmp;
		  *step_ramp= fabsf(go_angle/tmp);
  	}
    

 // ta=   fabsf(go_angle/max_rpm_speed);
  //tmp = ta/ts;
 // *cnt_ramp=  (uint32_t)tmp;
  //*step_ramp= fabsf(go_angle/tmp);

  printf("STEP RAMP =%.6f, Count RAMP=%lu ,  STEP CRUISE =%.6f, Count Cruise =%lu \r\n",
			*step_ramp, *cnt_ramp, *step_cruise, *cnt_cuise);
  

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
