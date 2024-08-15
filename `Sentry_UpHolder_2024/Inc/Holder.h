/**
  ******************************************************************************
  * @file    holder.h
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
#ifndef _HOLDER_H
#define _HOLDER_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "motor.h"
#include "message.h"
#include "pid.h"
	 
/* typedef -------------------------------------------------------------------*/
typedef struct _Yaw_t
{
    PID_IncrementType PidInc_Speed;
    PID_IncrementType PidInc_Position;
    PID_IncrementType_section PidInc_Position_Section;
	
	  PID_AbsoluteType PidAbs_Speed;
	  PID_AbsoluteType PidAbs_Position;
	  PID_AbsoluteType_section_kp PidAbs_Position_Section;
	
		PID_AbsoluteType PidAbs_Speed_Shoot;
	  PID_AbsoluteType PidAbs_Position_Shoot;
	
	  M_CIRCLE_T Position;
	
    int32_t TarAngle;  
	  int32_t IMU_Angle;
	  int32_t IMU_Angle_Lpf;
	  int16_t Angle; 
	  int32_t TarSpeed;
	  int32_t IMU_Speed;
	  int32_t IMU_Speed_Lpf;
	  int16_t Speed;
    int32_t Output;
    int32_t Output_Lpf;
	  int8_t Direction;
}Yaw_t;

typedef struct _Pitch_t
{
    PID_IncrementType PidInc_Speed;
    PID_IncrementType PidInc_Position;	
	  PID_IncrementType_section PidInc_Position_Section;

	  PID_AbsoluteType PidAbs_Speed;
	  PID_AbsoluteType PidAbs_Position;
	  PID_AbsoluteType_section PidAbs_Position_Section;
	
		PID_AbsoluteType PidAbs_Speed_Shoot;
	  PID_AbsoluteType PidAbs_Position_Shoot;
	
    M_CIRCLE_T Position;
	
    int32_t TarAngle;
	  int32_t IMU_Angle;
	  int32_t IMU_Angle_Lpf;
    int16_t Angle;
		int32_t Motor_Angle;
		int32_t Motor_Angle_Lpf;
	  int32_t IMU_Speed;
	  int32_t IMU_Speed_Lpf;
	  int32_t TarSpeed;
	  int16_t Speed;
		int32_t Motor_Speed;
	  int16_t Current;
    int32_t Output;
    int32_t Output_Lpf;
		int8_t Direction;
}Pitch_t;

typedef struct _Holder_t
{
    Yaw_t Yaw_0x20A;
	  Pitch_t Pitch_0x20B;
    uint8_t CanData[8];
}Holder_t;

/* define --------------------------------------------------------------------*/

/* variables -----------------------------------------------------------------*/
extern Holder_t Holder;
extern float vision_pitch, vision_yaw, vision_z;
extern M_CIRCLE_T v_imu_yaw;
extern float gimbal_vision_yaw_tar_anger,gimbal_vision_pitch_tar_anger;
extern float vision_yaw_navigate;
/* function ------------------------------------------------------------------*/
void Holder_Protect(void);  /* ÔÆÌ¨±£»¤ */
void Holder_Process(REMOTE_DATA_T RemoteMsg);
void Holder_PidInit(void);
void Holder_PID_Calculate(REMOTE_DATA_T RemoteMsg);
void Holder_Cruise(void);
void Holder_Forward(void);
void Holder_CanTransmit(void);
void IMU_GetHolderData(void);
void Holder_Manual(REMOTE_DATA_T RemoteMsg);
void Holder_Vision_deal(void);
void Holder_Vision(REMOTE_DATA_T RemoteMsg);
void Holder_Vision_Manual(REMOTE_DATA_T RemoteMsg);
void Holder_follow(void);
void Holder_Navigate(void);
float GetDirAngle_Holder(uint8_t ID);
#ifdef __cplusplus
}
#endif

#endif /* */
  
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/

