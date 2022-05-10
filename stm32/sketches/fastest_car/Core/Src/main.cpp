/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//////////////////////////// REAR WHEELS ////////////////////////////
#define MOTOR_ENCODER_TIMER_PERIOD 		65535

#define REAR_WHEEL_RADIUS_CM			3.3
#define REAR_WHEEL_ROTATION_DISTANCE	(2 * 3.141 * REAR_WHEEL_RADIUS_CM)
#define ENCODER_PULSES_PER_ROTATION		1500
#define DISTANCE_PER_ENCODER_PULSE		(REAR_WHEEL_ROTATION_DISTANCE / ENCODER_PULSES_PER_ROTATION)
#define ENCODER_DELTA_BOUND				50

#define STRAIGHT_MOTOR_PWM						3000
#define LEFT_TURNING_MOTOR_PWM					2800
#define RIGHT_TURNING_MOTOR_PWM					2700
//////////////////////////// FRONT WHEELS & STEERING ////////////////////////////

// Steering
#define CENTER_STEERING_PWM				75
#define STEERING_LEFT_EXTREME_DELTA		26
//#define STEERING_RIGHT_EXTREME_DELTA	38
#define STEERING_RIGHT_EXTREME_DELTA	16


#define EXTREME_RIGHT_STEERING_PWM 		(CENTER_STEERING_PWM + STEERING_RIGHT_EXTREME_DELTA)
#define EXTREME_LEFT_STEERING_PWM		(CENTER_STEERING_PWM - STEERING_LEFT_EXTREME_DELTA)
#define STEERING_BUFFER_TIME_MS			500

// Fixed Turning radius measured from the middle of the car body
#define TARGET_LEFT_TURNING_RADIUS			26
//#define TARGET_RIGHT_TURNING_RADIUS			27
#define TARGET_RIGHT_TURNING_RADIUS			56

// Distance between the front wheels, measured from the middle of each wheel
#define FRONT_WHEEL_TO_WHEEL_CM			17

#define LEFT_TURNING_90_OUTER_OFFSET		3.0
#define LEFT_TURNING_180_OUTER_OFFSET		0.0
#define RIGHT_TURNING_90_OUTER_OFFSET		0.0
#define RIGHT_TURNING_180_OUTER_OFFSET		0.3

#define LEFT_TURNING_INNER_RADIUS		(TARGET_LEFT_TURNING_RADIUS - (FRONT_WHEEL_TO_WHEEL_CM/2))
#define LEFT_TURNING_OUTER_RADIUS		(TARGET_LEFT_TURNING_RADIUS + (FRONT_WHEEL_TO_WHEEL_CM/2))
#define RIGHT_TURNING_INNER_RADIUS		(TARGET_RIGHT_TURNING_RADIUS - (FRONT_WHEEL_TO_WHEEL_CM/2))
#define RIGHT_TURNING_OUTER_RADIUS		(TARGET_RIGHT_TURNING_RADIUS + (FRONT_WHEEL_TO_WHEEL_CM/2))

#define LEFT_TURNING_INNER_TICKS ((2 * 3.142 * (double)LEFT_TURNING_INNER_RADIUS) * ((double)cmd.angle_deg / 360.0)) / DISTANCE_PER_ENCODER_PULSE
#define LEFT_TURNING_OUTER_TICKS ((2 * 3.142 * (double)LEFT_TURNING_OUTER_RADIUS) * ((double)cmd.angle_deg / 360.0)) / DISTANCE_PER_ENCODER_PULSE
#define RIGHT_TURNING_INNER_TICKS ((2 * 3.142 * (double)RIGHT_TURNING_INNER_RADIUS) * ((double)cmd.angle_deg / 360.0)) / DISTANCE_PER_ENCODER_PULSE
#define RIGHT_TURNING_OUTER_TICKS ((2 * 3.142 * (double)RIGHT_TURNING_OUTER_RADIUS) * ((double)cmd.angle_deg / 360.0)) / DISTANCE_PER_ENCODER_PULSE

#define LEFT_TURNING_90_OUTER_TICKS_OFFSET		(LEFT_TURNING_90_OUTER_OFFSET / DISTANCE_PER_ENCODER_PULSE)
#define LEFT_TURNING_180_OUTER_TICKS_OFFSET		(LEFT_TURNING_180_OUTER_OFFSET / DISTANCE_PER_ENCODER_PULSE)
#define RIGHT_TURNING_90_OUTER_TICKS_OFFSET		(RIGHT_TURNING_90_OUTER_OFFSET / DISTANCE_PER_ENCODER_PULSE)
#define RIGHT_TURNING_180_OUTER_TICKS_OFFSET	(RIGHT_TURNING_180_OUTER_OFFSET / DISTANCE_PER_ENCODER_PULSE)

//////////////////////////// RPI TO STM32 COMMANDS ////////////////////////////

// Movement State Machine
typedef uint8_t 	MOVEMENT_DIRECTION_TYPE;
typedef uint16_t 	MOVEMENT_DISTANCE_TYPE;
typedef uint16_t	MOVEMENT_TURNING_DEGREE_TYPE;

const MOVEMENT_DIRECTION_TYPE MOVE_FORWARD 			= 1;
const MOVEMENT_DIRECTION_TYPE MOVE_BACKWARD 		= 2;
const MOVEMENT_DIRECTION_TYPE MOVE_FORWARD_LEFT 	= 3;
const MOVEMENT_DIRECTION_TYPE MOVE_BACKWARD_LEFT 	= 4;
const MOVEMENT_DIRECTION_TYPE MOVE_FORWARD_RIGHT	= 5;
const MOVEMENT_DIRECTION_TYPE MOVE_BACKWARD_RIGHT	= 6;

struct Movement_Command
{
	MOVEMENT_DIRECTION_TYPE 		command;
	MOVEMENT_DISTANCE_TYPE 			distance_cm		= 0;
	MOVEMENT_TURNING_DEGREE_TYPE	angle_deg		= 0;
};

// RPI to STM32 Message size in bytes
#define RPI_TO_STM_MSG_SIZE				5
// Buffer to receive commands sent from RPI
volatile uint8_t rx_buffer[RPI_TO_STM_MSG_SIZE];

// RTOS queue for storing movement commands received from the RPI
osMessageQueueId_t movement_command_queue;

/////////////////////// STM32 to RPI COMMAND STATUS ///////////////////////

typedef uint8_t COMMAND_STATUS_TYPE;

const COMMAND_STATUS_TYPE COMMAND_NOT_READY_STATUS		= 0;
const COMMAND_STATUS_TYPE COMMAND_READY_STATUS			= 1;
const COMMAND_STATUS_TYPE COMMAND_RECEIVED_STATUS		= 2;
const COMMAND_STATUS_TYPE COMMAND_EXECUTING_STATUS		= 3;

volatile COMMAND_STATUS_TYPE stm32_status = COMMAND_NOT_READY_STATUS;

volatile bool started_run = false;

//////////////////////////// INFRARED ////////////////////////////

#define IR_ADC_CHANNEL_COUNT		2
#define IR_CONVERSION_PER_SAMPLE	100
#define IR_BUFFER_SIZE				(IR_ADC_CHANNEL_COUNT * IR_CONVERSION_PER_SAMPLE)
#define IR_ADC_REF 					3.3
#define IR_ADC_STEPS 				4096

typedef uint16_t INFRARED_DISTANCE_TYPE;

volatile INFRARED_DISTANCE_TYPE adcResultsDMA[IR_BUFFER_SIZE];

// Struct for passing infrared reading from task to USART task
struct Infrared_Data
{
	INFRARED_DISTANCE_TYPE left_distance;
	INFRARED_DISTANCE_TYPE right_distance;
};

// RTOS queue for storing infrared data to be sent to the RPI
osMessageQueueId_t infrared_data_queue;

//////////////////////////// ULTRASONIC ////////////////////////////
volatile uint16_t Ultrasonic_IC_Val1 = 0;
volatile uint16_t Ultrasonic_IC_Val2 = 0;
volatile uint16_t Ultrasonic_Difference = 0;
volatile uint8_t Ultrasonic_Is_First_Captured = 0;  // is the first value captured ?

typedef double ULTRASONIC_DISTANCE_TYPE;
const ULTRASONIC_DISTANCE_TYPE ULTRASONIC_OFFSET = 1;

// RTOS queue for storing infrared data to be sent to the RPI
osMessageQueueId_t ultrasonic_readings_queue;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void delay_microsecond (uint16_t time);
void HCSR04_Read (void);
void set_motor_forward(void);
void set_motor_backward(void);
void set_motor_stop(void);
void set_motor_brake(void);
uint32_t get_encoder_delta(uint32_t count1, uint32_t count2, TIM_HandleTypeDef* hal_tim);
void set_steering_forward(void);
void set_steering_left(void);
void set_steering_right(void);
void disconnect_steering(void);
void command_to_encoder_counts(Movement_Command cmd, uint32_t* left_motor_target_ptr, uint32_t* right_motor_target_ptr);
void calibrate_steering_center(MOVEMENT_DIRECTION_TYPE direction);
bool encoder_within_bounds(uint32_t target, uint32_t delta);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MovementTask */
osThreadId_t MovementTaskHandle;
const osThreadAttr_t MovementTask_attributes = {
  .name = "MovementTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RPITransmitTask */
osThreadId_t RPITransmitTaskHandle;
const osThreadAttr_t RPITransmitTask_attributes = {
  .name = "RPITransmitTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow7,
};
/* Definitions for UltrasonicTask */
osThreadId_t UltrasonicTaskHandle;
const osThreadAttr_t UltrasonicTask_attributes = {
  .name = "UltrasonicTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow6,
};
/* Definitions for InfraredTask */
//osThreadId_t InfraredTaskHandle;
//const osThreadAttr_t InfraredTask_attributes = {
//  .name = "InfraredTask",
//  .stack_size = 128 * 4,
//  .priority = (osPriority_t) osPriorityLow6,
//};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM3_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
//static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void movement_task(void *argument);
void RPI_Transmit_Task(void *argument);
void ultrasonic_task(void *argument);
//void infrared_task(void *argument);
void DMATransferComplete(DMA_HandleTypeDef* hdma);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
//INFRARED_DISTANCE_TYPE adc_to_distance(uint32_t raw);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Callback function for when USART DMA transmit is done
void DMATransferComplete(DMA_HandleTypeDef* hdma)
{
		// Disable the DMA
		huart3.Instance->CR3 &= ~USART_CR3_DMAT;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//	if (hadc == &hadc1)
//	{
//		uint32_t left_ir_sum = 0;
//		uint32_t right_ir_sum = 0;
//		for (uint8_t i = 0; i < IR_CONVERSION_PER_SAMPLE; i++)
//		{
//			uint8_t idx = (i * 2);
//			left_ir_sum += adcResultsDMA[idx];
//			right_ir_sum += adcResultsDMA[idx + 1];
//		}
//
//		Infrared_Data data;
//		data.left_distance = adc_to_distance((uint32_t)left_ir_sum/IR_CONVERSION_PER_SAMPLE);
//		data.right_distance = adc_to_distance((uint32_t)right_ir_sum/IR_CONVERSION_PER_SAMPLE);
//
//		osMessageQueuePut(infrared_data_queue, &data, 0U, 0U);
//	}
//}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	// if the interrupt source is for the ultrasonic echo
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
		if (Ultrasonic_Is_First_Captured==0) // if the first value is not captured
		{
			Ultrasonic_IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3); // read the first value
			Ultrasonic_Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (Ultrasonic_Is_First_Captured==1)   // if the first is already captured
		{
			Ultrasonic_IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (Ultrasonic_IC_Val2 > Ultrasonic_IC_Val1)
			{
				Ultrasonic_Difference = Ultrasonic_IC_Val2-Ultrasonic_IC_Val1;
			}
			else if (Ultrasonic_IC_Val1 > Ultrasonic_IC_Val2)
			{
				Ultrasonic_Difference = (0xffff - Ultrasonic_IC_Val1) + Ultrasonic_IC_Val2;
			}
			ULTRASONIC_DISTANCE_TYPE distance = ((double)Ultrasonic_Difference * 0.034/2) + ULTRASONIC_OFFSET;
			osMessageQueuePut(ultrasonic_readings_queue, &distance, 0U, 0U);
			Ultrasonic_Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC3);
		}
	}
}

// Callback function when USART receives data from the RPI.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if (huart == &huart3)
	{
		bool valid_cmd = false;
		Movement_Command move_msg;
		char c1 = rx_buffer[0];
		char c2 = rx_buffer[1];
		char d[4];
		memcpy(d, (void*)&rx_buffer[2], 3);
		d[3] = '\0';
		if (c1 == 'F')
		{
			if (c2 == 'W')
			{
				move_msg.command = MOVE_FORWARD;
				move_msg.distance_cm = strtol(d, NULL, 10);
				valid_cmd = true;
			}
			else if (c2 == 'L')
			{
				move_msg.command = MOVE_FORWARD_LEFT;
				move_msg.angle_deg = strtol(d, NULL, 10);
				valid_cmd = true;
			}
			else if (c2 == 'R')
			{
				move_msg.command = MOVE_FORWARD_RIGHT;
				move_msg.angle_deg = strtol(d, NULL, 10);
				valid_cmd = true;
			}
		}
		else if (c1 == 'B')
		{
			if (c2 == 'W')
			{
				move_msg.command = MOVE_BACKWARD;
				move_msg.distance_cm = strtol(d, NULL, 10);
				valid_cmd = true;
			}
			else if (c2 == 'L')
			{
				move_msg.command = MOVE_BACKWARD_LEFT;
				move_msg.angle_deg = strtol(d, NULL, 10);
				valid_cmd = true;
			}
			else if (c2 == 'R')
			{
				move_msg.command = MOVE_BACKWARD_RIGHT;
				move_msg.angle_deg = strtol(d, NULL, 10);
				valid_cmd = true;
			}
		}
		if (valid_cmd)
		{
			started_run = true;
			stm32_status = COMMAND_RECEIVED_STATUS;
			// Put the command message into the queue
			osMessageQueuePut(movement_command_queue, &move_msg, 0U, 0U);
		}
		HAL_UART_Receive_IT(&huart3, (uint8_t*)rx_buffer, RPI_TO_STM_MSG_SIZE);
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
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_DMA_Init();
  // IMPORTANT! UART_Init must come after DMA_Init.
  // CubeMX has a bug where the order of function call is reversed!!!!
  MX_USART3_UART_Init();
  // IMPORTANT! ADC1_Init must come after DMA_Init.
  // CubeMX has a bug where the order of function call is reversed!!!!
//  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  // Init steering PWM
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  // Left Motor Encoder
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  // Right Motor Encoder
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  // Set up the Motors PWM
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

  // Init DMA send data callback
  HAL_DMA_RegisterCallback(&hdma_usart3_tx, HAL_DMA_XFER_CPLT_CB_ID, &DMATransferComplete);

  // Init the buffer to receive commands from RPI
  HAL_UART_Receive_IT(&huart3, (uint8_t*)rx_buffer, RPI_TO_STM_MSG_SIZE);

  // Init interrupts for the timer attached to the ultrasonic sensor echo pin
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);

  // We have intialized all the hardware components
  stm32_status = COMMAND_READY_STATUS;
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  movement_command_queue = osMessageQueueNew(20, sizeof(Movement_Command), NULL);
  ultrasonic_readings_queue = osMessageQueueNew(10, sizeof(ULTRASONIC_DISTANCE_TYPE), NULL);
  infrared_data_queue = osMessageQueueNew(10, sizeof(Infrared_Data), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MovementTask */
  MovementTaskHandle = osThreadNew(movement_task, NULL, &MovementTask_attributes);

  /* creation of RPITransmitTask */
  RPITransmitTaskHandle = osThreadNew(RPI_Transmit_Task, NULL, &RPITransmitTask_attributes);

  /* creation of UltrasonicTask */
  UltrasonicTaskHandle = osThreadNew(ultrasonic_task, NULL, &UltrasonicTask_attributes);

  /* creation of InfraredTask */
//  InfraredTaskHandle = osThreadNew(infrared_task, NULL, &InfraredTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
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
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 320-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 2-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 8000-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MotorA_HBridge_1_Pin|MotorA_HBridge_2_Pin|MotorB_HBridge_1_Pin|MotorB_HBridge_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Ultrasonic_Trig_GPIO_Port, Ultrasonic_Trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MotorA_HBridge_1_Pin MotorA_HBridge_2_Pin MotorB_HBridge_1_Pin MotorB_HBridge_2_Pin */
  GPIO_InitStruct.Pin = MotorA_HBridge_1_Pin|MotorA_HBridge_2_Pin|MotorB_HBridge_1_Pin|MotorB_HBridge_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Ultrasonic_Trig_Pin */
  GPIO_InitStruct.Pin = Ultrasonic_Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Ultrasonic_Trig_GPIO_Port, &GPIO_InitStruct);

}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_ADC1_Init(void)
//{
//
//  /* USER CODE BEGIN ADC1_Init 0 */
//
//  /* USER CODE END ADC1_Init 0 */
//
//  ADC_ChannelConfTypeDef sConfig = {0};
//
//  /* USER CODE BEGIN ADC1_Init 1 */
//
//  /* USER CODE END ADC1_Init 1 */
//  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
//  */
//  hadc1.Instance = ADC1;
//  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
//  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
//  hadc1.Init.ScanConvMode = ENABLE;
//  hadc1.Init.ContinuousConvMode = ENABLE;
//  hadc1.Init.DiscontinuousConvMode = DISABLE;
//  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
//  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
//  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//  hadc1.Init.NbrOfConversion = 2;
//  hadc1.Init.DMAContinuousRequests = DISABLE;
//  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
//  if (HAL_ADC_Init(&hadc1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_10;
//  sConfig.Rank = 1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_11;
//  sConfig.Rank = 2;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN ADC1_Init 2 */
//
//  /* USER CODE END ADC1_Init 2 */
//
//}

/* USER CODE BEGIN 4 */

// Delay the microprocessor. The Timer1 has been set to 1MHZ by the prescalar.
// Thus 1 tick is equal to 1 microsecond.
void delay_microsecond (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	uint16_t dur = time * 2;
	while (__HAL_TIM_GET_COUNTER (&htim1) < dur);
}

void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(Ultrasonic_Trig_GPIO_Port, Ultrasonic_Trig_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_microsecond(10);  // wait for 10 us
	HAL_GPIO_WritePin(Ultrasonic_Trig_GPIO_Port, Ultrasonic_Trig_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC3); // Enable interrupts
}

// Set the H-Bridge to move the wheels forward
void set_motor_forward(void)
{
  	// Left Motor
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	// Right Motor
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

// Set the H-Bridge to move the wheels backward
void set_motor_backward(void)
{
  	// Left Motor
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	// Right Motor
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

// Set the H-Bridge to disconnect the wheels from the motor
void set_motor_stop(void)
{
  	// Left Motor
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	// Right Motor
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

// Set the H-Bridge to disconnect the wheels from the motor
void set_motor_brake(void)
{
  	// Left Motor
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	// Right Motor
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

// Turn the steering forward
void set_steering_forward(void)
{
	htim4.Instance->CCR4 = CENTER_STEERING_PWM;
	osDelay(STEERING_BUFFER_TIME_MS);
	disconnect_steering();
}

// Turn the steering right
void set_steering_right(void)
{
	htim4.Instance->CCR4 = EXTREME_RIGHT_STEERING_PWM;
	osDelay(STEERING_BUFFER_TIME_MS);
	disconnect_steering();
}

// Turn the steering left
void set_steering_left(void)
{
	htim4.Instance->CCR4 = EXTREME_LEFT_STEERING_PWM;
	osDelay(STEERING_BUFFER_TIME_MS);
	disconnect_steering();
}

void disconnect_steering(void)
{
	htim4.Instance->CCR4 = 0;
	osDelay(250);
}

void calibrate_steering_center(MOVEMENT_DIRECTION_TYPE direction)
{
	if (direction == MOVE_FORWARD_LEFT || MOVE_BACKWARD_LEFT)
	{
		htim4.Instance->CCR4 = EXTREME_RIGHT_STEERING_PWM;
		osDelay(500);
		htim4.Instance->CCR4 = EXTREME_LEFT_STEERING_PWM;
		osDelay(500);
	}
	else
	{
		htim4.Instance->CCR4 = EXTREME_LEFT_STEERING_PWM;
		osDelay(500);
		htim4.Instance->CCR4 = EXTREME_RIGHT_STEERING_PWM;
		osDelay(500);
	}
	htim4.Instance->CCR4 = CENTER_STEERING_PWM;
	osDelay(500);
	disconnect_steering();
}

// From 2 timer counts, calculate the delta/timer counts that passed between them.
// @param count1: The first timer count
// @param count2: The second timer count
// @param count3: Pointer to the timer handle
// @return The number of counts that elapsed between count1 and count2
uint32_t get_encoder_delta(uint32_t count1, uint32_t count2, TIM_HandleTypeDef* hal_tim)
{
	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(hal_tim))
	{
		if (count2 <= count1)
		{
			return count1 - count2;
		}
		else
		{
			return (MOTOR_ENCODER_TIMER_PERIOD - count2) + count1;
		}
	}
	else
	{
		if (count2 >= count1)
		{
			return count2 - count1;
		}
		else
		{
			return (MOTOR_ENCODER_TIMER_PERIOD - count1) + count2;
		}
	}
}

// Convert command to encoder wheel encoder count targets.
// Takes care of both straight line movement (where both left & right wheel
// encoder counts are equal), as well as turning, where the
// inner and outer wheels encoder counts are different.
void command_to_encoder_counts(Movement_Command cmd, uint32_t* left_motor_target_ptr, uint32_t* right_motor_target_ptr)
{
	*left_motor_target_ptr = 0;
	*right_motor_target_ptr = 0;
	if (cmd.command == MOVE_FORWARD || cmd.command == MOVE_BACKWARD)
	{
		uint32_t ticks = (uint32_t)((double)cmd.distance_cm / (double)DISTANCE_PER_ENCODER_PULSE);
		*left_motor_target_ptr = ticks;
		*right_motor_target_ptr = ticks;
		return;
	}
	else
	{
		if (cmd.command == MOVE_FORWARD_LEFT)
		{
			if (cmd.angle_deg == 90)
			{
				*left_motor_target_ptr = (uint32_t)LEFT_TURNING_INNER_TICKS;
				*right_motor_target_ptr = (uint32_t)(LEFT_TURNING_OUTER_TICKS + LEFT_TURNING_90_OUTER_TICKS_OFFSET);
			}
			else if (cmd.angle_deg == 180)
			{
				*left_motor_target_ptr = (uint32_t)LEFT_TURNING_INNER_TICKS;
				*right_motor_target_ptr = (uint32_t)(LEFT_TURNING_OUTER_TICKS + LEFT_TURNING_180_OUTER_TICKS_OFFSET);
			}
		}
		else if(cmd.command == MOVE_FORWARD_RIGHT)
		{
			if (cmd.angle_deg == 90)
			{
				*right_motor_target_ptr = RIGHT_TURNING_INNER_TICKS;
				*left_motor_target_ptr = (uint32_t)(RIGHT_TURNING_OUTER_TICKS + RIGHT_TURNING_90_OUTER_TICKS_OFFSET);
			}
			else if (cmd.angle_deg == 180)
			{
				*right_motor_target_ptr = RIGHT_TURNING_INNER_TICKS;
				*left_motor_target_ptr = (uint32_t)(RIGHT_TURNING_OUTER_TICKS + RIGHT_TURNING_180_OUTER_TICKS_OFFSET);
			}
		}
	}
}

inline bool encoder_within_bounds(uint32_t target, uint32_t delta)
{
	return (delta >= target) || (target - delta <= ENCODER_DELTA_BOUND);

}

// Convert ADC value to distance for IR sensor
//INFRARED_DISTANCE_TYPE adc_to_distance(uint32_t raw)
//{
//	  double voltage = raw * IR_ADC_REF / IR_ADC_STEPS;
//	  return (INFRARED_DISTANCE_TYPE)(29.988 * pow(voltage, -1.173));
//}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_movement_task */
/**
* @brief Function implementing the MovementTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_movement_task */
void movement_task(void *argument)
{
  /* USER CODE BEGIN movement_task */
	uint32_t 	encoder_A_count1, encoder_A_count2, encoder_A_count_delta,
				encoder_B_count1, encoder_B_count2, encoder_B_count_delta;
//	uint32_t 	encoder_A_ticks1, encoder_A_ticks2, encoder_A_ticks_delta,
//				encoder_B_ticks1, encoder_B_ticks2, encoder_B_ticks_delta;

	uint32_t left_motor_target 	= 0;
	uint32_t right_motor_target = 0;
	uint32_t left_motor_delta 	= 0;
	uint32_t right_motor_delta 	= 0;

	Movement_Command move_cmd;
	bool toMove = false;

	// Disconnect the Motor First
	set_motor_stop();

	// Set the front wheels to point forward
	set_steering_forward();

	// Set initial values for count and ticks for both encoders
	encoder_A_count1 = __HAL_TIM_GET_COUNTER(&htim2);
//	encoder_A_ticks1 = HAL_GetTick();
	encoder_B_count1 = __HAL_TIM_GET_COUNTER(&htim3);
//	encoder_B_ticks1 = HAL_GetTick();

  /* Infinite loop */
  for(;;)
  {
	  if (!toMove)
	  {
		  osStatus_t status = osMessageQueueGet(movement_command_queue, &move_cmd, NULL, 0);
		  if (status == osOK)
		  {

			  // Set H-bridge
			  // Change the H-Bridge settings
			  if (move_cmd.command == MOVE_BACKWARD || move_cmd.command == MOVE_BACKWARD_LEFT
					  || move_cmd.command == MOVE_BACKWARD_RIGHT)
			  {
				  set_motor_backward();
			  }
			  else
			  {
				  set_motor_forward();
			  }

			  // Set steering
			  if (move_cmd.command == MOVE_FORWARD || move_cmd.command == MOVE_BACKWARD)
			  {
				  set_steering_forward();
			  }
			  else if (move_cmd.command == MOVE_FORWARD_LEFT || move_cmd.command == MOVE_BACKWARD_LEFT)
			  {
				  set_steering_left();
			  }
			  else
			  {
				  set_steering_right();
			  }

			  // Calculate the ticks set points for both motors
			  command_to_encoder_counts(move_cmd, &left_motor_target, &right_motor_target);

	  	  	  // Reset the other values
			  left_motor_delta 	= 0;
			  right_motor_delta 	= 0;

	  		  // Reset the encoder value
	  	  	  encoder_A_count1 = __HAL_TIM_GET_COUNTER(&htim2);
//	  	  	  encoder_A_ticks1 = HAL_GetTick();
	  	  	  encoder_B_count1 = __HAL_TIM_GET_COUNTER(&htim3);
//	  	  	  encoder_B_ticks1 = HAL_GetTick();


			  toMove = true;
		  }
	  }

	  if (toMove)
	  {
		  	  stm32_status = COMMAND_EXECUTING_STATUS;

		  	  encoder_A_count2 = __HAL_TIM_GET_COUNTER(&htim2);
//		  	  encoder_A_ticks2 = HAL_GetTick();
		  	  encoder_B_count2 = __HAL_TIM_GET_COUNTER(&htim3);
//		  	  encoder_B_ticks2 = HAL_GetTick();

		  	  // Calculate the ticks delta
//		  	  encoder_A_ticks_delta = encoder_A_ticks2 - encoder_A_ticks1;
//		  	  encoder_B_ticks_delta = encoder_B_ticks2 - encoder_B_ticks1;
		  	  // Calculate the counter delta
		  	  encoder_A_count_delta = get_encoder_delta(encoder_A_count1, encoder_A_count2, &htim2);
		  	  encoder_B_count_delta = get_encoder_delta(encoder_B_count1, encoder_B_count2, &htim3);

		  	  left_motor_delta += encoder_A_count_delta;
		  	  right_motor_delta += encoder_B_count_delta;

		  	  // The left motor leads the right motor
		  	  if (move_cmd.command == MOVE_FORWARD || move_cmd.command == MOVE_BACKWARD)
		  	  {

			  	  if (encoder_within_bounds(left_motor_target, left_motor_delta)) // Stop both motors
			  	  {
			  		  left_motor_delta = 0;
			  		  right_motor_delta = 0;
			  		  left_motor_target = 0;
			  		  right_motor_target = 0;
					  // Stop both motors
					  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, (uint16_t)0);
					  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, (uint16_t)0);
					  if (osMessageQueueGetCount(movement_command_queue) == 0)
					  {
						  stm32_status = COMMAND_READY_STATUS;
					  }
					  toMove = false;
			  	  }
			  	  else
			  	  {
			  		 __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, (uint16_t)STRAIGHT_MOTOR_PWM);
			  		 __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, (uint16_t)STRAIGHT_MOTOR_PWM);
			  	  }
		  	  }
		  	  else // The inner motor follows the outer motor
		  	  {
		  		  // Target is the outer wheel
		  		  uint32_t ticks_target = 0;
		  		  uint32_t ticks_done = 0;
		  		  if (move_cmd.command == MOVE_FORWARD_LEFT)
		  		  {
		  			  ticks_done = right_motor_delta;
		  			  ticks_target = right_motor_target;
		  		  }
		  		  else if (move_cmd.command == MOVE_FORWARD_RIGHT)
		  		  {
		  			  ticks_done = left_motor_delta;
		  			  ticks_target = left_motor_target;
		  		  }

		  		if (encoder_within_bounds(ticks_target, ticks_done))
		  		{
		  			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, (uint16_t)0);
		  			__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, (uint16_t)0);
			  		  left_motor_delta = 0;
			  		  right_motor_delta = 0;
			  		  left_motor_target = 0;
			  		  right_motor_target = 0;
			  		if (osMessageQueueGetCount(movement_command_queue) == 0)
			  		{
//			  			calibrate_steering_center(move_cmd.command);
			  			stm32_status = COMMAND_READY_STATUS;
			  		}
			  			toMove = false;
		  		}
		  		else
		  		{
		  			if (move_cmd.command == MOVE_FORWARD_LEFT)
		  			{
			  			 __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, (uint16_t)LEFT_TURNING_MOTOR_PWM);
			  			 __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, (uint16_t)LEFT_TURNING_MOTOR_PWM);
		  			}
		  			else if (move_cmd.command == MOVE_FORWARD_RIGHT)
		  			{
			  			 __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, (uint16_t)RIGHT_TURNING_MOTOR_PWM);
			  			 __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, (uint16_t)RIGHT_TURNING_MOTOR_PWM);
		  			}
		  		}
		  	  }
	  }

	  // Restart the process again
  	  encoder_A_count1 = __HAL_TIM_GET_COUNTER(&htim2);
//  	  encoder_A_ticks1 = HAL_GetTick();
  	  encoder_B_count1 = __HAL_TIM_GET_COUNTER(&htim3);
//  	  encoder_B_ticks1 = HAL_GetTick();

  	  osDelay(25);
  }
  /* USER CODE END movement_task */
}

/* USER CODE BEGIN Header_RPI_Transmit_Task */
/**
* @brief Function implementing the RPITransmitTask thread. This task transmit data
* 	     from the STM32 to RPI.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RPI_Transmit_Task */
void RPI_Transmit_Task(void *argument)
{
  /* USER CODE BEGIN RPI_Transmit_Task */
	char command_str[10]; // Include null-terminating character

	osStatus_t ultrasonic_osStatus;
	ULTRASONIC_DISTANCE_TYPE ultrasonic_distance;
	char ultrasonic_str[10]; // Include null-terminating character

	osStatus_t infrared_osStatus;
	Infrared_Data infrared_data;
	char infrared_str[15]; // Include null-terminating character

	// String to transmit to the STM32
	char transmit_msg[35];

  /* Infinite loop */
  for(;;)
  {
	  // Encode STM32 status
	  sprintf(command_str, "CM%.3hu", (uint16_t)stm32_status);

	  ultrasonic_osStatus = osMessageQueueGet(ultrasonic_readings_queue, &ultrasonic_distance, NULL, 0);
	  if (ultrasonic_osStatus != osOK)
	  {
		  strcpy(ultrasonic_str, "USxxx"); // Copy over null-terminating character as well
	  }
	  else
	  {
		  sprintf(ultrasonic_str, "US%.3hu", (uint16_t)ultrasonic_distance);
	  }

	  infrared_osStatus = osMessageQueueGet(infrared_data_queue, &infrared_data, NULL, 0);
	  if (infrared_osStatus != osOK)
	  {
		  strcpy(infrared_str, "ILxxx,IRxxx"); // Copy over null-terminating character as well
	  }
	  else
	  {
		  sprintf(infrared_str, "IL%.3hu,IR%.3hu", infrared_data.left_distance, infrared_data.right_distance);
	  }

	  // Concat all information into a string
	  memset(transmit_msg, 0, sizeof(transmit_msg));
	  sprintf(transmit_msg, "%s,%s,%s", command_str, ultrasonic_str, infrared_str);
	  strcat(transmit_msg, "\n");

	  // Enable the DMA to start a transfer process
	  huart3.Instance->CR3 |= USART_CR3_DMAT;
	  HAL_DMA_Start_IT(&hdma_usart3_tx, (uint32_t)transmit_msg,
	  		  		(uint32_t)&huart3.Instance->DR, strlen(transmit_msg) + 1);

	  if (started_run)
	  {
		  osDelay(1000);
	  }
	  else
	  {
		  osDelay(100);
	  }
  }
  /* USER CODE END RPI_Transmit_Task */
}

/* USER CODE BEGIN Header_ultrasonic_task */
/**
* @brief Function implementing the UltrasonicTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ultrasonic_task */
void ultrasonic_task(void *argument)
{
  /* USER CODE BEGIN ultrasonic_task */
  /* Infinite loop */
  for(;;)
  {
	  HCSR04_Read();
	  if (started_run)
	  {
		  osDelay(1000);
	  }
	  else
	  {
		  osDelay(100);
	  }
  }
  /* USER CODE END ultrasonic_task */
}

/* USER CODE BEGIN Header_infrared_task */
/**
* @brief Function implementing the InfraredTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_infrared_task */
//void infrared_task(void *argument)
//{
//  /* USER CODE BEGIN infrared_task */
//  /* Infinite loop */
//  for(;;)
//  {
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcResultsDMA, IR_BUFFER_SIZE);
//    osDelay(500);
//  }
//  /* USER CODE END infrared_task */
//}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

