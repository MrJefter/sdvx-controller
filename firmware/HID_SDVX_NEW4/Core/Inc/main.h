/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN4_Pin GPIO_PIN_1
#define BTN4_GPIO_Port GPIOA
#define BTN4_EXTI_IRQn EXTI1_IRQn
#define BTN3_Pin GPIO_PIN_2
#define BTN3_GPIO_Port GPIOA
#define BTN3_EXTI_IRQn EXTI2_IRQn
#define BTN2_Pin GPIO_PIN_3
#define BTN2_GPIO_Port GPIOA
#define BTN2_EXTI_IRQn EXTI3_IRQn
#define FXL_Pin GPIO_PIN_4
#define FXL_GPIO_Port GPIOA
#define FXL_EXTI_IRQn EXTI4_IRQn
#define BTN1_Pin GPIO_PIN_5
#define BTN1_GPIO_Port GPIOA
#define BTN1_EXTI_IRQn EXTI9_5_IRQn
#define KNOBLA_Pin GPIO_PIN_6
#define KNOBLA_GPIO_Port GPIOA
#define KNOBLB_Pin GPIO_PIN_7
#define KNOBLB_GPIO_Port GPIOA
#define FXR_Pin GPIO_PIN_9
#define FXR_GPIO_Port GPIOA
#define FXR_EXTI_IRQn EXTI9_5_IRQn
#define START_Pin GPIO_PIN_10
#define START_GPIO_Port GPIOA
#define START_EXTI_IRQn EXTI15_10_IRQn
#define KNOBRA_Pin GPIO_PIN_6
#define KNOBRA_GPIO_Port GPIOB
#define KNOBRB_Pin GPIO_PIN_7
#define KNOBRB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
