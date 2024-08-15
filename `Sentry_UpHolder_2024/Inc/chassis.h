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
#ifndef _CHASSIS_H
#define _CHASSIS_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "motor.h"
#include "message.h" 
#include "pid.h"
#include <stdbool.h>
/* typedef -------------------------------------------------------------------*/

typedef struct _MoveData_t
{
    float Left;    
    float Front;
    float ClockWise;
}MoveData_t;

typedef struct _ChassisSpeed
{
	  float x;
    float y;	
    float yaw;
}CHASSIS_SPEED_T;

typedef struct _M3508_T
{
		int16_t Speed_Rx;
	  float Speed;
	  float Speed_Lpf;
		float TarSpeed;
		float TarSpeed_Lpf;
	  int16_t Current_Rx;
		float Current;
    float Current_Lpf;   
		float TarCurrent;
	  float TarCurrent_Lpf; 
		int16_t Output;
		int16_t Output_Lpf;
	
		PID_IncrementType PidInc_Current;
		PID_IncrementType PidInc_Speed;
}M3508_T;

typedef struct _SuperCap
{
    uint8_t  sup_state;
    uint16_t cap_vol;
    uint16_t in_vol;
    uint16_t in_cur;
    uint32_t chassis_power;
    uint32_t target_power;
    uint8_t  can_data[8];
    uint8_t cap_switch_cmd;
}SUPER_CAP_T; /*< ������������ */

typedef struct Chassis_T
{
    M3508_T M3508[5];	
	  uint8_t Can_Data[8];
	  SUPER_CAP_T SuperCap;
	  MoveData_t MoveData;
		int32_t IMU_Angle;
	  int32_t IMU_Angle_Lpf;
		int32_t IMU_Speed;
		int32_t IMU_Speed_Lpf;
}Chassis_T;


/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
#define CHASSIS_SUPERCAP  (0x00) //���ݹ���
#define CHASSIS_POWER     (0x01) //��Դ����ģ�鹩��
#define HolderMode    (0x00) //���̷��Ӿ�ģʽ
#define ChassisMode      (0x01) //�����Ӿ�ģʽ

extern uint8_t control_state;
extern Chassis_T Chassis;
extern CHASSIS_SPEED_T ChassisSpeed; 
/* function ------------------------------------------------------------------*/
void Chassis_PidInit(void);  //PID��ʼ��
void Chassis_Process_Tmp(REMOTE_DATA_T RemoteMsg, Chassis_Data_T ChassisMsg);
void Chassis_Process(REMOTE_DATA_T RemoteMsg, Chassis_Data_T ChassisMsg);  //���̽���
float RosControl(void); //ROS����
void GetRemoteMoveData(REMOTE_DATA_T RemoteMsg); 
void Chassis_Protect(void); //���̱���
void Chassis_PidCalculate(void);  //����PID����
float Follow_Holder(float dir_angle);
float DynamicSpin(void);
void Chassis_Kinematic(void);  //�����˶�ѧ
void WheelSpeedConstrait(float* tmp_wheel_speed); 
float Speed_tunning(void);
void Chassis_ChooseMode(REMOTE_DATA_T RemoteMsg);
void Chassis_CanTransmit(void);
static void SuperCapCanTransmit(void);
void IMU_GetChassisData(Chassis_Data_T* ChassisMsg);

#ifdef __cplusplus
}
#endif

#endif /* */

