/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim9;

extern TIM_HandleTypeDef htim10;

extern TIM_HandleTypeDef htim14;

/* USER CODE BEGIN Private defines */
typedef struct _rate_t
{
    uint16_t CAN1_Tx_Rate;
    uint8_t CAN1_0x201_Rate;
    uint8_t CAN1_0x202_Rate;
    uint8_t CAN1_0x203_Rate;
    uint8_t CAN1_0x204_Rate;
	  uint8_t CAN1_0x208_Rate;
    uint8_t CAN1_0x209_Rate;
    uint8_t CAN1_0x20A_Rate;
	  uint8_t CAN1_0x20B_Rate;
    
    uint16_t CAN2_Tx_Rate;
    uint8_t CAN2_0x201_Rate;
    uint8_t CAN2_0x202_Rate;
    uint8_t CAN2_0x203_Rate;
    uint8_t CAN2_0x204_Rate;
    
    uint8_t  DR16_Rate;
    int16_t  MPU6500_Value;
    float    HI216_Value;
    uint16_t CAMERA_Rate;
}rate_t;

typedef struct _obser_t
{
    rate_t Tx;
    rate_t Rx;
}obser_t;

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM9_Init(void);
void MX_TIM10_Init(void);
void MX_TIM14_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */
extern obser_t Observer;
extern uint8_t flag,shoot_flag;
extern uint16_t hh1,hh2,hflag;
void TIM1_PWMSTART(void);
void TIM9_PWMSTART(void);
void TIM1_SetPWMPluse(uint16_t pluse_left, uint16_t pluse_right);
void TIM9_SetPWMPluse(uint16_t pluse_left, uint16_t pluse_right);
void TIM10_START(void);
void TIM14_START(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

