#ifndef _CHASSIS_CONFIG_H
#define _CHASSIS_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include <stdint.h>
#include "config.h"
#include <stdbool.h>
	 
/******************************************************************************
  * @brief  发射机构控制相关参数
  * @attention 每次只能定义一个波形参数，调试完务必注释掉debug相关定义
  ****************************************************************************/

/* 底盘状态参数 */
int8_t move_state = 0;	           /* 底盘运动状态 */
int16_t chassis_shift_cnt = 0;     /* 底盘状态切换计数 */
float clockwise = 0;               /* 底盘旋转角速度 */
float front_temp = 0;              /* 底盘向前速度 */
float left_temp = 0;	             /* 底盘向左速度 */ 
float fmax_wheel_speed = 8000.0f;  /* M3508限定最大转速：rpm */
/* 底盘物理参数 */
float Chassis_r = 0.2765;              /* 底盘半径r */
float Omin_r = 0.067;          /* 全向轮半径r */
float GAIN_F = 0.00037; 
/*
   速度换算：
   1rpm = 0.1047rad/s
   全向轮转速为1rpm，换算成全向实际速度(沿辊子方向)rad/s如下：
   1rpm * 0.1047 * 0.067 °= 0.007105，再把减速比算进去，可得：
   GAIN_F = 0.007105*(187/3591) = 0.00037
*/

/* 底盘电机PID */ 
//速度环
	float M3508_Speed_kp = 9.0f;  //8.0
  float M3508_Speed_ki = 0.1f; //0.01
  float M3508_Speed_kd = 12.0f; //12.0
//电流环
	float M3508_Current_kp = 0.8f;  //0.8f
  float M3508_Current_ki = 0.1f;  //0.1
  float M3508_Current_kd = 0.1f;  //0.1
	 
  float Chassis_Out_Limit = 30000.0f;

#ifdef __cplusplus
}
#endif

#endif
	
	

