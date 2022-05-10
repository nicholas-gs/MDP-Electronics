/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IRSensor_Left_Pin GPIO_PIN_0
#define IRSensor_Left_GPIO_Port GPIOC
#define IRSensor_Right_Pin GPIO_PIN_1
#define IRSensor_Right_GPIO_Port GPIOC
#define MotorA_HBridge_1_Pin GPIO_PIN_2
#define MotorA_HBridge_1_GPIO_Port GPIOA
#define MotorA_HBridge_2_Pin GPIO_PIN_3
#define MotorA_HBridge_2_GPIO_Port GPIOA
#define MotorB_HBridge_1_Pin GPIO_PIN_4
#define MotorB_HBridge_1_GPIO_Port GPIOA
#define MotorB_HBridge_2_Pin GPIO_PIN_5
#define MotorB_HBridge_2_GPIO_Port GPIOA
#define MotorB_Encoder_Signal_1_Pin GPIO_PIN_6
#define MotorB_Encoder_Signal_1_GPIO_Port GPIOA
#define MotorB_Encoder_Signal_2_Pin GPIO_PIN_7
#define MotorB_Encoder_Signal_2_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_10
#define LED_GPIO_Port GPIOE
#define Ultrasonic_Echo_Pin GPIO_PIN_13
#define Ultrasonic_Echo_GPIO_Port GPIOE
#define Steering_PWM_Pin GPIO_PIN_15
#define Steering_PWM_GPIO_Port GPIOD
#define MotorA_PWM_Pin GPIO_PIN_6
#define MotorA_PWM_GPIO_Port GPIOC
#define MotorB_PWM_Pin GPIO_PIN_7
#define MotorB_PWM_GPIO_Port GPIOC
#define MotorA_Encoder_Signal_2_Pin GPIO_PIN_15
#define MotorA_Encoder_Signal_2_GPIO_Port GPIOA
#define STM32_RPI_TX_Pin GPIO_PIN_10
#define STM32_RPI_TX_GPIO_Port GPIOC
#define STM32_RPI_RX_Pin GPIO_PIN_11
#define STM32_RPI_RX_GPIO_Port GPIOC
#define Ultrasonic_Trig_Pin GPIO_PIN_4
#define Ultrasonic_Trig_GPIO_Port GPIOD
#define MotorA_Encoder_Signal_1_Pin GPIO_PIN_3
#define MotorA_Encoder_Signal_1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
