/**
  ******************************************************************************
  * @file    
  * @author  sy
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
#ifndef _VISION_H
#define _VISION_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include "usart.h"
#include "message.h"
#include <stdbool.h>	 
#define VISION_RECEIVE_DATA_SIZE (200) // 接收数据大小

/* 哨兵状态 */
#define Navig   (0x00)	  /* 自动导航模式 */
#define Shoot_Out (0x01)  /* 击打前哨站 */
#define Guard_Area (0x02) /* 回防模式 */
#define Shoot_Robo (0x03) /* 击打地面单位 */
	 
/*调试模式*/
//#define DEBUG
	 
/* 射频控制 */
#define Low_Speed (0x00)
#define Fast_Speed (0x01)
/* typedef -------------------------------------------------------------------*/
typedef struct
{
    float Pitch_Angle;	
    float Yaw_Angle;
    float Distance;
	  float speed_x;
	  float speed_y;
	  float speed_yaw;	
	  uint8_t firesuggestion;
	  uint8_t isSpining;	
		uint8_t isSpining_Vision;	
		uint8_t isNavigating;
}RECV_DATA_T;

typedef struct
{
    RECV_DATA_T recv_data;
    uint8_t index;
    uint8_t buf[VISION_RECEIVE_DATA_SIZE];
}vData_t;

/* 定义联合体 */
typedef union
{
		uint8_t yaw[2];
	  int16_t Angle;
}vYaw_t;

typedef union
{
		uint8_t pitch[2];
	  int16_t Angle;
}vPitch_t;

/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
extern float PI1;
extern float vData_x, vData_y, vData_z;
extern float vData_x_speed, vData_y_speed, vData_yaw_speed;
extern vData_t vData;
extern	float Pitch_IMU_Angle, Vision_Pitch_IMU_Angle;    /* 俯仰角度 */
extern float Yaw_IMU_Angle, Vision_Yaw_IMU_Angle;      /* 偏航角度 */ 
extern uint8_t Robot_ID;
extern int32_t err_cnt, err_record;
/* function ------------------------------------------------------------------*/
void Vision_RecvData(uint8_t byte);
void Vision_Get_cmdkeyboard(void);
void Vision_SendData(void);
void Recieve_Vision_Data(uint8_t *Recieve_Buf,uint16_t Len);
void Vision_Protect(void);
void Vision_Judge(void);
//static void Holder_CoordinateTransform(void);
#ifdef __cplusplus
}
#endif

#endif /* */
  
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
