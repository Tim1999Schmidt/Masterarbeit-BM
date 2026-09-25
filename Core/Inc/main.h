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
#include "stm32c0xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define OSC_32_OUT_Pin GPIO_PIN_15
#define OSC_32_OUT_GPIO_Port GPIOC
#define TEMP_Pin GPIO_PIN_1
#define TEMP_GPIO_Port GPIOA
#define HALL_OUT_Pin GPIO_PIN_2
#define HALL_OUT_GPIO_Port GPIOA
#define CONNECT_Z_Pin GPIO_PIN_5
#define CONNECT_Z_GPIO_Port GPIOA
#define U_Z__Pin GPIO_PIN_6
#define U_Z__GPIO_Port GPIOA
#define STATUS_LED1_Pin GPIO_PIN_7
#define STATUS_LED1_GPIO_Port GPIOA
#define NSLEEP_Pin GPIO_PIN_0
#define NSLEEP_GPIO_Port GPIOB
#define SEL1_Pin GPIO_PIN_1
#define SEL1_GPIO_Port GPIOB
#define SEL2_Pin GPIO_PIN_2
#define SEL2_GPIO_Port GPIOB
#define STATUS_LED2_Pin GPIO_PIN_8
#define STATUS_LED2_GPIO_Port GPIOA
#define ERR_EXT_OUT_Pin GPIO_PIN_11
#define ERR_EXT_OUT_GPIO_Port GPIOA
#define ERR_LOC_OUT_Pin GPIO_PIN_12
#define ERR_LOC_OUT_GPIO_Port GPIOA
#define SEL3_Pin GPIO_PIN_3
#define SEL3_GPIO_Port GPIOB
#define SEL4_Pin GPIO_PIN_4
#define SEL4_GPIO_Port GPIOB
#define ERRQ_RES_Pin GPIO_PIN_5
#define ERRQ_RES_GPIO_Port GPIOB
#define ERRQ_EXT_Pin GPIO_PIN_6
#define ERRQ_EXT_GPIO_Port GPIOB
#define ERRQ_Pin GPIO_PIN_7
#define ERRQ_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
