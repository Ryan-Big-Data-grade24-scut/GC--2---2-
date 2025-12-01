/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "VL53L0X.h"
#include "gc_pid.h"
#include "stdio.h"
#include "VL6180X.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
  uint16_t ecd; uint16_t last_ecd;
  float speed_real; float speed_tar;
  GPIO_TypeDef *IN1_port; uint16_t IN1_pin; //两个IN用来控制正反转
  GPIO_TypeDef *IN2_port; uint16_t IN2_pin;
  int32_t pwm_output;
  TIM_HandleTypeDef *PWM_htim; uint32_t PWM_Channel;
  TIM_HandleTypeDef *ecd_htim;
}motor_t;

typedef struct{
  int8_t mode;

  float speed_Kp;
  float speed_Ki;
  float speed_Kd;
  float speed_iMax;
  float speed_iMin;
  
  float angle_A_Kp;
  float angle_A_Ki;
  float angle_A_Kd;
  float angle_A_iMax;
  float angle_A_iMin;

  float angle_S_Kp;
  float angle_S_Ki;
  float angle_S_Kd;
  float angle_S_iMax;
  float angle_S_iMin;

  float track_Kp;
  float track_Ki;
  float track_Kd;

}PID_t;

typedef struct{
  float pitch;
  float roll;
  float yaw;

}IMU_t;


//PID模型结构�?
// extern P rtP; //输入
// extern ExtU rtU;  //参数
// extern ExtY rtY;  //输出


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_CPR 1040
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef VL53L0X_I2C_Handler;
statInfo_t_VL53L0X statInfo;
statInfo_t_VL53L0X statInfo2;

vl6180x_t tof1;
vl6180x_t tof2;

char vlBuffer[64];

uint8_t uart1_tx_resetZaxis[3] = {0xFF,0xAA,0x52};
uint8_t uart1_tx_setWidth[3] = {0xFF,0xAA,0x81};
uint8_t uart1_index = 0;
uint8_t uart1_rx_buffer[64];
uint8_t imu_buffer[16];
uint8_t uart2_rx_buffer[8];

uint16_t tof_dis1 = 8192;
uint16_t tof_dis2 = 8192;
uint16_t tof_dis1_l = 8192;
uint16_t tof_dis2_l = 8192;
uint16_t tof_dis1_ll = 8192;
uint16_t tof_dis2_ll = 8192;


float ang_tar = 0.0f;
float ang_real = 0.0f;
float ang_ori = 0.0f;

IMU_t imu_data = {0.0f,0.0f,0.0f};

PID_t pid_param = {
  1,   //mode
  //0.017,0.005,0.0001,500,-500,  //speed 只在mode=1时生效
	0.02,0.04,0.001,500,-500,
  8.8,0.15,0.005,1000,-1000,  //angle_A 只在mode=2时生效
  0.08,0.02,0.001,1000,-1000,  //angle_S 只在mode=2时生效
  0.12,0.1,0.0001   //巡线用pid，一直生效
};
//0.037
//9.5
//0.12

motor_t motor_left = {
  0,0,
  0,0,
  AIN1_GPIO_Port, AIN1_Pin, //左A右B
  AIN2_GPIO_Port, AIN2_Pin,
  0,
  &htim3,TIM_CHANNEL_1,
  &htim1
};

motor_t motor_right = {
  0,0,
  0,0,
  BIN2_GPIO_Port, BIN2_Pin,
  BIN1_GPIO_Port, BIN1_Pin,
  0,
  &htim3,TIM_CHANNEL_2,
  &htim2
};

float track_state[8];
float line_track = 0;

int tim4_cnt = 0;

int8_t CAR_STATE = 1;

uint16_t case2_cnt = 0;
uint16_t case3_cnt = 0;
uint16_t case4_cnt = 0;
uint16_t case5_cnt = 0;
uint16_t case6_cnt = 0;
uint16_t case7_cnt = 0;

uint16_t tof_cnt = 0;

float tmp_tar = 0.0f;
int uart_cnt = 0;

int16_t stop_cnt = 0;
uint16_t stop_code = 0;

uint16_t imu_cnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void motor_updEcd(motor_t *motor){
  motor->last_ecd = motor->ecd;
  motor->ecd = __HAL_TIM_GET_COUNTER(motor->ecd_htim);
  int32_t delta = motor->ecd - motor->last_ecd;
  if(delta > (int32_t)(MOTOR_CPR*0.7)){
    delta -= 65535;
  }else if(delta < (int32_t)(-MOTOR_CPR*0.7 )){
    delta += 65535;
  }
  motor->speed_real = (float)delta * 360 / MOTOR_CPR * 100;
}

void PID_updParam(PID_t *pid){
  
  rtP.angle_A_Kp = pid->angle_A_Kp;
  rtP.angle_A_Ki = pid->angle_A_Ki;
  rtP.angle_A_Kd = pid->angle_A_Kd;
  rtP.angle_A_iMax = pid->angle_A_iMax;
  rtP.angle_A_iMin = pid->angle_A_iMin;
  
  rtP.angle_S_Kp = pid->angle_S_Kp;
  rtP.angle_S_Ki = pid->angle_S_Ki;
  rtP.angle_S_Kd = pid->angle_S_Kd;
  rtP.angle_S_iMax = pid->angle_S_iMax;
  rtP.angle_S_iMin = pid->angle_S_iMin;
  
  rtP.speed_Kp = pid->speed_Kp;
  rtP.speed_Ki = pid->speed_Ki;
  rtP.speed_Kd = pid->speed_Kd;
  rtP.speed_iMax = pid->speed_iMax;
  rtP.speed_iMin = pid->speed_iMin;

  rtP.track_Kp = pid->track_Kp;
  rtP.track_Ki = pid->track_Ki;
  rtP.track_Kd = pid->track_Kd;
}

void PID_updInput(){
  rtU.ctrl_mode = pid_param.mode;
  rtU.angle_tar = ang_tar;
  rtU.angle_real = imu_data.yaw;
  rtU.leftRpm_real = motor_left.speed_real;
  rtU.rightRpm_real = motor_right.speed_real;
  rtU.leftRpm_tar = motor_left.speed_tar;
  rtU.rightRpm_tar = motor_right.speed_tar;
  rtU.track_in = line_track;
  rtU.track_tar = 0.0f;
}

bool CRC_check(uint8_t *buffer){
  uint8_t sum = 0;
  for(int i=0; i<=9; i++){
      sum += buffer[i];
  }
  return sum == buffer[10];
}

void IMU_getAng(uint8_t* buffer){
	if(buffer[1] == 0x53){
		uint16_t roll = (buffer[3] << 8) | buffer[2];
		uint16_t pitch = (buffer[5] << 8) | buffer[4];
		uint16_t yaw = (buffer[7] << 8) | buffer[6];
		imu_data.roll = (float)roll / 32768.0f * 180.0f;
		imu_data.pitch = (float)pitch / 32768.0f * 180.0f;
		imu_data.yaw = (float)yaw / 32768.0f * 180.0f;
  }
}

void tof_init(){
  initVL53L0X(1, &hi2c1);
  setSignalRateLimit(250);
  setVcselPulsePeriod(VcselPeriodPreRange, 18);
	setVcselPulsePeriod(VcselPeriodFinalRange, 18);
	setMeasurementTimingBudget(190 * 1000UL);
  startContinuous(0);
  HAL_Delay(20);
  initVL53L0X(1,&hi2c2);
  setSignalRateLimit(250);
  setVcselPulsePeriod(VcselPeriodPreRange, 18);
  setVcselPulsePeriod(VcselPeriodFinalRange, 18);
  setMeasurementTimingBudget(190 * 1000UL);
  startContinuous(0);
}

void tof_getDis(){
  VL53L0X_I2C_Handler = hi2c1;
  tof_dis1_ll = tof_dis1_l;
  tof_dis1_l = tof_dis1;
  tof_dis1 = readRangeContinuousMillimeters(&statInfo);
  //HAL_Delay(5);
  VL53L0X_I2C_Handler = hi2c2;
  tof_dis2_ll = tof_dis2_l;
  tof_dis2_l = tof_dis2;
  tof_dis2 = readRangeContinuousMillimeters(&statInfo2);
}

// void tof_init(){
//   vl6180x_init(&tof1 ,&hi2c1);
//   vl6180x_configure_default(&tof1);
// //  vl6180x_write_reg(&tof1, SYSRANGE__MAX_CONVERGENCE_TIME, 30);
// //  vl6180x_write_reg_16bit(&tof1, SYSALS__INTEGRATION_PERIOD, 50);
//   vl6180x_set_timeout(&tof1,500);
// //  vl6180x_stop_continuous(&tof1);
// //  HAL_Delay(300);
//   // vl6180x_start_range_continuous(&tof1, 500);
  
//   vl6180x_init(&tof2 ,&hi2c2);
//   vl6180x_configure_default(&tof2);
// //  vl6180x_write_reg(&tof2, SYSRANGE__MAX_CONVERGENCE_TIME, 30);
// //  vl6180x_write_reg_16bit(&tof2, SYSALS__INTEGRATION_PERIOD, 50);
//   vl6180x_set_timeout(&tof2,500);
// //  vl6180x_stop_continuous(&tof2);
// //  HAL_Delay(300);
//   // vl6180x_start_range_continuous(&tof2, 500);
// }

//void tof_getDis(){
//  tof_dis1 = vl6180x_read_range_continuous_millimeters(&tof1);
//  // tof_dis1 = vl6180x_read_range_single_millimeters(&tof1);
//	HAL_Delay(20);
//  tof_dis2 = vl6180x_read_range_continuous_millimeters(&tof2);
//  // tof_dis2 = vl6180x_read_range_single_millimeters(&tof2);
//}

void setMotor_CW(motor_t *motor){
  HAL_GPIO_WritePin(motor->IN1_port, motor->IN1_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(motor->IN2_port, motor->IN2_pin, GPIO_PIN_SET);
}

void setMotor_CCW(motor_t *motor){
  HAL_GPIO_WritePin(motor->IN1_port, motor->IN1_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor->IN2_port, motor->IN2_pin, GPIO_PIN_RESET);
}

void drive_motor(motor_t *motor, int32_t speed){
  if(speed < -1000){  //pid输出钳位
    speed = -1000;
  }else if(speed > 1000){
    speed = 1000;
  }
  if(speed < 0){
    setMotor_CCW(motor);
    __HAL_TIM_SET_COMPARE(motor->PWM_htim, motor->PWM_Channel, -(uint32_t)speed);
  }else{
    setMotor_CW(motor);
    __HAL_TIM_SET_COMPARE(motor->PWM_htim, motor->PWM_Channel, (uint32_t)speed);
  }
}

float ffabs(float a){
  return a > 0 ? a : -a;
}

void motor_updPID(){
//  if(ffabs(motor_left.speed_tar - motor_left.speed_real) > 5.0f){
    motor_left.pwm_output += (int32_t)rtY.leftRpm_output;
    motor_right.pwm_output += (int32_t)rtY.rightRpm_output;
//  }
  drive_motor(&motor_left, motor_left.pwm_output);
  drive_motor(&motor_right, motor_right.pwm_output);
}



void lineTrack_update(){   //从上到下为编�?8�?1
  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 1){        //PA11
    if(track_state[7]<20)track_state[7]+=2.4;
  }else{
    if(track_state[7]>0)track_state[7]-=1;
  }
  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == 1){  //PA12
    if(track_state[6]<20)track_state[6]+=2.1;
  }else{
    if(track_state[6]>0)track_state[6]-=1.2;
  }
  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == 1){  //PA15
    if(track_state[5]<20)track_state[5]+=2;
  }else{
    if(track_state[5]>0)track_state[5]--;
  }
  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 1){   //PB3
    if(track_state[4]<20)track_state[4]+=2;
  }else{
    if(track_state[4]>0)track_state[4]-=1.3;
  }

  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 1){   //PB4
    if(track_state[3]<20)track_state[3]+=2;
  }else{
    if(track_state[3]>0)track_state[3]-=1.3;
  }

  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 1){   //PB5
    if(track_state[2]<20)track_state[2]+=2;
  }else{
    if(track_state[2]>0)track_state[2]--;
  }
    
  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 1){   //PB1
    if(track_state[1]<20)track_state[1]+=2.1;
  }else{
    if(track_state[1]>0)track_state[1]-=1.2;
  }
    
  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 1){   //PB0
    if(track_state[0]<20)track_state[0]+=2.4;
  }else{
    if(track_state[0]>0)track_state[0]-=1;
  }
  
  if(track_state[0] && track_state[1] && track_state[2] && track_state[3] && track_state[4] && track_state[5] && track_state[6] && track_state[7]){
//    if(stop_flag == 0){
//      stop_flag = 1;
//    }
    stop_cnt++;
  }else{
    if(stop_cnt>0)stop_cnt -= 1;
  }
//  
  if(stop_cnt >= 45 && CAR_STATE == 1){
    stop_cnt = 0;
    stop_code ++;
  }

//  if(stop_flag){
//    stop_cnt ++;
//    if(stop_cnt == 200){
//      stop_flag = 0;
//      stop_cnt = 0;
//      if(track_state[0] && track_state[1] && track_state[2] && track_state[3] && track_state[4] && track_state[5] && track_state[6] && track_state[7]){
//        stop_code++;
//      }
//    }
//  }

  line_track = track_state[0]*-130 + track_state[1]*-65 + track_state[2]*-40 + 
								track_state[3]*-10 + track_state[4]*10 + track_state[5]*40+ 
								track_state[6]*65 + track_state[7]*130;
  if(line_track < -600000)line_track = -600000;
  if(line_track > 600000)line_track = 600000;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)

{

  /* USER CODE BEGIN 1 */
  
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_Delay(1400);
	

	tof_init();
  gc_pid_initialize();
  //htim3用于pwm输出，pwm范围�?1-1000
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  
  //htim4用于定时�?
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	
  
  HAL_UART_Transmit_IT(&huart1, uart1_tx_resetZaxis, 3);  //IMU初始�?
  HAL_UART_Transmit_IT(&huart1, uart1_tx_setWidth, 3);
  HAL_Delay(15);
	HAL_UART_Receive_IT(&huart1, uart1_rx_buffer, 1);
	
	HAL_UART_Receive_IT(&huart2, uart2_rx_buffer, 3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		ang_tar = 0;
//		 HAL_Delay(5000);
//     ang_tar = 90;
//		 HAL_Delay(5000);
//		 ang_tar = 0;
//		 HAL_Delay(5000);
//		 ang_tar = 270;
//		 HAL_Delay(5000);
//		 ang_tar = 0;
		
//		HAL_UART_Receive(&huart1, uart1_rx_buffer, 11, 100);
//		if(uart1_rx_buffer[0] == 0x55 && uart1_rx_buffer[1] == 0x53){
//      IMU_getAng();
//    }
     //int size = sprintf((char*)uart2_tx_buffer, "%f,%f \n", imu_data.yaw, ang_tar);
    //int size = sprintf((char*)uart2_tx_buffer, "%f,%d\n", motor_right.speed_real, motor_right.pwm_output);
    //HAL_UART_Transmit_IT(&huart2, uart2_tx_buffer, size);
    
    tof_getDis();
    switch(CAR_STATE){
    case 0:
        motor_right.speed_tar = 165;
        motor_left.speed_tar = -165;
        HAL_Delay(750);
        pid_param.mode = 2;
        ang_tar = 0.0f;
        motor_right.speed_tar = 0;
        motor_left.speed_tar = 0;
        break;
    case 1:        //直行    
        
    
        if((tof_dis2+tof_dis2_l+tof_dis2_ll)/3 <= 330){
            motor_right.speed_tar = 135;
            motor_left.speed_tar = -135;
            PID_updInput(); 
//            HAL_Delay(300);
//            motor_right.speed_tar = 0;
//            motor_left.speed_tar = 0;
//            HAL_Delay(300);
            //CAR_STATE = 2;
            CAR_STATE = 2;
            break;
        }
        
        if((tof_dis1+tof_dis1_l+tof_dis1_ll)/3 <= 330){  //970 210
            
            motor_right.speed_tar = 135;
            motor_left.speed_tar = -135;
            PID_updInput();
//            HAL_Delay(300);
//            motor_right.speed_tar = 0;
//            motor_left.speed_tar = 0;
//            HAL_Delay(300);
            //CAR_STATE = 5;
              CAR_STATE = 5;
            break;
        }
        break;
    case 2:        
        if(case2_cnt > 150){
          
          pid_param.mode = 2;
          tmp_tar = ang_ori+37.0f; 
          if(tmp_tar > 360)tmp_tar-=360;
          ang_tar = tmp_tar;
          case2_cnt = 0;
          HAL_Delay(100);
          pid_param.mode = 1;
            case2_cnt = 0;
            motor_right.speed_tar = 133;
            motor_left.speed_tar = -195;
            CAR_STATE = 3;
        }
        //if(case2_cnt > 450){
            
        //}
        break;
    case 3:        //绕障（右侧）
        if(case3_cnt > 150 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == 1){ 
            CAR_STATE = 4;
            case3_cnt = 0;
        }
        break;
    case 4:        //回转（右侧）
        HAL_Delay(50);
        pid_param.mode = 2;
        ang_tar = ang_ori;
        if(case4_cnt > 150){
            case4_cnt = 0;
            CAR_STATE = 1;
        }
        break;
    case 5:        //左侧转向
        if(case5_cnt > 150){
          pid_param.mode = 2;
          tmp_tar = ang_ori-35.0f; 
          if(tmp_tar < 0)tmp_tar+=360;
          ang_tar = tmp_tar;
          case5_cnt = 0;
          HAL_Delay(100);
          pid_param.mode = 1;
          case5_cnt = 0;
          motor_right.speed_tar = 195; 
          motor_left.speed_tar = -133;
          CAR_STATE = 6;
        }
        //if(case5_cnt > 450){
            
        //}
        break;
    case 6:        //绕障（左侧）
        if(case6_cnt > 150 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 1){ 
            CAR_STATE = 7;
            case6_cnt = 0;
        }
        break;
    case 7:        //回转（左侧）
        HAL_Delay(50);
        pid_param.mode = 2;
        ang_tar = ang_ori;
        if(case7_cnt > 450){
            case7_cnt = 0;
            CAR_STATE = 1;
        }
        break;
    case 8:
        pid_param.mode = 1;
        motor_right.speed_tar = 180;
        motor_left.speed_tar = -180;
        HAL_Delay(1300);
        motor_right.speed_tar = 0;
        motor_left.speed_tar = 0;
        HAL_Delay(100);
        pid_param.mode = 2;
        //tmp_tar = ang_ori-90.0f;  
        //if(tmp_tar < 0)tmp_tar+=360;
        ang_tar = 90.0f;
        HAL_Delay(600);
        //CAR_STATE = 9;
        pid_param.mode = 1;
        motor_right.speed_tar = 120;
        motor_left.speed_tar = -120;
        HAL_Delay(1100);
        CAR_STATE = 1;
        break;
    case 9:
        motor_right.speed_tar = 150;
        motor_left.speed_tar = -150;
        HAL_Delay(1200);
        motor_right.speed_tar = 0;
        motor_left.speed_tar = 0;
        HAL_Delay(200);
        pid_param.mode = 2;
        ang_tar = 0.0f;
        HAL_Delay(800);
        pid_param.mode = 1;
        CAR_STATE = 1;
        break;
    default:
        pid_param.mode = 1;
        motor_right.speed_tar = 0;
        motor_left.speed_tar = 0;
        break;
    }

    //HAL_Delay(7);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(huart->Instance == USART1){
    uart_cnt++;
    if(uart1_index == 0){
      if(uart1_rx_buffer[0] == 0x55){
        imu_buffer[uart1_index++] = uart1_rx_buffer[0];
      }else{
        uart1_index = 0;
      }
    }else if(uart1_index == 1){
      if(uart1_rx_buffer[0] == 0x53){
        imu_buffer[uart1_index++] = uart1_rx_buffer[0];
      }else{
        uart1_index = 0;
      }
    }else if(uart1_index == 10){
      imu_buffer[uart1_index] = uart1_rx_buffer[0];
      if(CRC_check(imu_buffer)){
        IMU_getAng(imu_buffer);
      }
      uart1_index = 0;
    }else{
      imu_buffer[uart1_index++] = uart1_rx_buffer[0];
    }
    HAL_UART_Receive_IT(&huart1, uart1_rx_buffer, 1);
  }
  if(huart->Instance == USART2){
    //if(uart2_rx_buffer[0] == 0xAA && uart2_rx_buffer[1] == 0x01 && uart2_rx_buffer[2] == 0x55){
    //&& imu_data.yaw<200 && imu_data.yaw > 160
      if(imu_cnt == 3 && CAR_STATE == 1 && imu_data.yaw<200 && imu_data.yaw > 160 ){
        CAR_STATE = 8;
      }
    //}
    HAL_UART_Receive_IT(&huart2, uart2_rx_buffer, 3);
  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim->Instance == TIM4){
    if(stop_code == 2 ){
//      if(imu_data.yaw < 20 ||  imu_data.yaw > 340){
        CAR_STATE = 0;
        stop_code = 1;
//      }else{
//        CAR_STATE = 9;
//        stop_code = 1;
//      }
    }
//    if(stop_code == 3){
//      CAR_STATE = 0;
//    }
		switch(CAR_STATE){
			case 0:
				case2_cnt = 0;
				break;
      case 1:
        if(imu_data.yaw > 80 && imu_data.yaw < 100){
          imu_cnt = 2;
        }
        if(imu_cnt == 2 && imu_data.yaw > 160 && imu_data.yaw < 200){
          imu_cnt = 3;
        }
        pid_param.mode = 1;
        ang_real = imu_data.yaw;
				ang_ori = ang_real;
        lineTrack_update();
        

          motor_right.speed_tar = 255+rtY.track_output;
          motor_left.speed_tar = -255+rtY.track_output;
        
        break;
			case 2:
				case2_cnt ++;
				break;
      case 3:
        case3_cnt ++;
        break;
			case 4:
				case4_cnt ++;
				break;
      case 5:
        case5_cnt ++;
        break;
      case 6:
        case6_cnt ++;
        break;
      case 7:
        case7_cnt ++;
        break;
			default:
				break;
		}
		
    tim4_cnt ++;
    if(tof_cnt < 500){tof_cnt++;}
    if(tim4_cnt == 10){
			
      tim4_cnt = 0;
      motor_updEcd(&motor_left);
      motor_updEcd(&motor_right);
      PID_updParam(&pid_param);
      PID_updInput();
      gc_pid_step();
      motor_updPID();
      //drive_motor(&motor_left, 100);
      // setMotor_CCW(&motor_left);
      // __HAL_TIM_SET_COMPARE(motor_left.PWM_htim, motor_left.PWM_Channel, 100);
    }
  }
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
