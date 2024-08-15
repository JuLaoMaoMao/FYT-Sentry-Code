/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define IMU_TX_Pin GPIO_PIN_0
#define IMU_TX_GPIO_Port GPIOA
#define IMU_RX_Pin GPIO_PIN_1
#define IMU_RX_GPIO_Port GPIOA
#define Snail1_Pin GPIO_PIN_2
#define Snail1_GPIO_Port GPIOA
#define Snail2_Pin GPIO_PIN_3
#define Snail2_GPIO_Port GPIOA
#define Snail3_Pin GPIO_PIN_13
#define Snail3_GPIO_Port GPIOE
#define Snail4_Pin GPIO_PIN_14
#define Snail4_GPIO_Port GPIOE
#define CS1_ACCEL_Pin GPIO_PIN_10
#define CS1_ACCEL_GPIO_Port GPIOB
#define CS1_GYRO_Pin GPIO_PIN_11
#define CS1_GYRO_GPIO_Port GPIOB
#define JUDGE_TX_Pin GPIO_PIN_8
#define JUDGE_TX_GPIO_Port GPIOD
#define JUDGE_RX_Pin GPIO_PIN_9
#define JUDGE_RX_GPIO_Port GPIOD
#define VISION_TX_Pin GPIO_PIN_6
#define VISION_TX_GPIO_Port GPIOC
#define VISION_RX_Pin GPIO_PIN_7
#define VISION_RX_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
