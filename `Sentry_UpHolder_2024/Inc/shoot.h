/**
  ******************************************************************************
  * @file    
  * @author  zz
  * @brief   
  * @date     
  ******************************************************************************
  * @attention
  *
  * Copyright (c) CSU_RM_FYT
  * All rights reserved.
  *
  * This software component is licensed by SY under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _SHOOT_H
#define _SHOOT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "message.h"
#include "motor.h"
#include "pid.h"
/* typedef -------------------------------------------------------------------*/
typedef struct _SHOOT_T
{
      M_CIRCLE_T Mcircle_Position;
 
	    float TarAngle;
      float Angle;
		  int16_t Speed;
	    int16_t RmfSpeed; /* 递推均值滤波 */ 
	    int16_t TarSpeed;
      int16_t TarSpeed_Lpf;
	    int16_t Output;
      int16_t Output_Lpf;
	    uint8_t Can_Data[8];
	
      PID_IncrementType PidInc_Position;
      PID_IncrementType PidInc_Speed;
	
	    PID_AbsoluteType  PidAbs_Position;
	    PID_AbsoluteType  PidAbs_Speed;
}SHOOT_T;

typedef struct _SHOOT_3508_T
{
      M_CIRCLE_T Mcircle_Position;
 
	    float TarAngle;
      float Angle;
		  int16_t Speed;
	    int16_t RmfSpeed; /* 递推均值滤波 */ 
	    int16_t TarSpeed;
      int16_t TarSpeed_Lpf;
	    int16_t Output;
      int16_t Output_Lpf;
	    uint8_t Can_Data[8];
	
      PID_IncrementType PidInc_Position;
      PID_IncrementType PidInc_Speed;
	
	    PID_AbsoluteType  PidAbs_Position;
	    PID_AbsoluteType  PidAbs_Speed;
}SHOOT_3508_T;
/* define --------------------------------------------------------------------*/
//#define SHOOT_TEST

/* variables -----------------------------------------------------------------*/
extern SHOOT_T Shoot_M2006;
extern SHOOT_3508_T Shoot_M3508[2];
extern int32_t shoot_heat_cnt,shoot_cal_heat;
/* function ------------------------------------------------------------------*/
void Shoot_PIDInit(void);
void Shoot_PidCalculation(REMOTE_DATA_T RemoteMsg);   /* 拨弹电机PID计算 */
void Shoot_Reset(void);    /* 拨弹电机保护 */
void Shoot_HeatControl(void);
void Shoot_GetHeat(void);
void Shoot_Process(REMOTE_DATA_T RemoteMsg);
void FrictionWheel_Process(REMOTE_DATA_T RemoteMsg); //摩擦轮进程
void Shoot_CanTransmit(void);
void Shoot_ChooseMode(REMOTE_DATA_T RemoteMsg);

#ifdef __cplusplus
}
#endif

#endif /* define*/
