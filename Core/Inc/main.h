/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void PID_InitAll(void);
static inline float PID_Output_To_Duty(PID_TypeDef *pid, float output);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CAP_I_Pin GPIO_PIN_1
#define CAP_I_GPIO_Port GPIOC
#define TEMP_Pin GPIO_PIN_2
#define TEMP_GPIO_Port GPIOC
#define V_CAP_Pin GPIO_PIN_3
#define V_CAP_GPIO_Port GPIOC
#define V_CHASSIS_Pin GPIO_PIN_0
#define V_CHASSIS_GPIO_Port GPIOA
#define I_REFEREE_Pin GPIO_PIN_1
#define I_REFEREE_GPIO_Port GPIOA
#define I_A_Pin GPIO_PIN_2
#define I_A_GPIO_Port GPIOA
#define A_H_Pin GPIO_PIN_8
#define A_H_GPIO_Port GPIOA
#define A_L_Pin GPIO_PIN_9
#define A_L_GPIO_Port GPIOA
#define B_H_Pin GPIO_PIN_10
#define B_H_GPIO_Port GPIOA
#define B_L_Pin GPIO_PIN_11
#define B_L_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define TIMERA_PERIOD   360U //pwm����tickֵ
#define CTRL_PWM_START_TICKS   (2U)  // �ɰ�ʾ�����۲������1~10 �����ԣ���ֹ0u��ʱ��򲻿�pwm���������ƫ��
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
