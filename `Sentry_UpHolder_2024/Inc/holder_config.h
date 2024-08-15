#ifndef _HOLDER_CONFIG_H
#define _HOLDER_CONFIG_H

#include <stdint.h>
#include "config.h"

/******************************************************************************
  * @brief  云台控制相关参数
  * @attention 双闭环控制 
  *     外环（角度环）采用变结构PI控制器，减小超调，加快响应
  *     内环（速度环）采用绝对式PID控制器
  *     调试方法：
  *         1.定义 #define HOLDER_DEBUG，可通过debug在线修改参数
  *         2.定义 #define HOLDER_PITCH_WAVE，可输出PITCH轴波形
  *         3.定义 #define HOLDER_YAW_WAVE, 可输出YAW轴波形
  *         4.定义 #define HOLDER_VISION_WAVE, 可输出视觉波形
  *         具体实现方法在holder.c中的static void Holder_Pid_Manual(void)函数中
  *
  *         每次只能定义一个波形参数，调试完务必注释掉debug相关定义
  ****************************************************************************/
/* Sentry data */

///*导航参数*/
///* 俯仰 */
//	float Pitch_Angle_kp = 0.22f; //0.18f
//	float Pitch_Angle_ki = 0.001f; //0.0015f
//	float Pitch_Angle_kd = 0.0f; //0.09f
//	float Pitch_Angle_kf = 2.0f;
//	float Pitch_Speed_kp = 23.0f; //15.0f
//	float Pitch_Speed_ki = 0.05f; //0.001
//	float Pitch_Speed_kd = 0.0f; //0.15f
//	float Pitch_Speed_kf = 0.0f;
//	
///* 偏航 */		
//	float Yaw_Angle_kp = 0.15f; //0.09//0.095
//	float Yaw_Angle_ki = 0.00f; //0
//	float Yaw_Angle_kd = 0.1f; //0.1   
//	float Yaw_Angle_kf = 0.0f;
//	float Yaw_Speed_kp = 75.0f; //20//32.5
//  float Yaw_Speed_ki = 0.00f; //3.0
//  float Yaw_Speed_kd = 0.0f; //1.0
//	float Yaw_Speed_kf = 1.0f;
	
/* 俯仰 */
	float Pitch_Speed_kp = 22.0f; //10.0f
	float Pitch_Speed_ki = 0.05f; //0.0f
	float Pitch_Speed_kd = 0.5f; //0.0f
	float Pitch_Speed_kf = 4.0f; //0.0f
	float Pitch_Angle_kp = 0.18f; //0.10f
	float Pitch_Angle_ki = 0.005f; //0.035f
	float Pitch_Angle_kd = 0.0f; //0.4f
	float Pitch_Angle_kf = 2.0f; //0.0f
/*自瞄参数*/
	float Pitch_Speed_kp_Shoot = 20.0f; //23.0f
	float Pitch_Speed_ki_Shoot = 0.1f; //0.05f
	float Pitch_Speed_kd_Shoot = 0.5f; //0.0f
	float Pitch_Speed_kf_Shoot = 4.0f; //0.0f
	float Pitch_Angle_kp_Shoot = 0.16f; //0.22f
	float Pitch_Angle_ki_Shoot = 0.005f; //0.001f
	float Pitch_Angle_kd_Shoot = 0.0f; //0.09f
	float Pitch_Angle_kf_Shoot = 2.0f; //2.0f

	
/* 偏航 */		
	float Yaw_Speed_kp = 30.0f; //50
  float Yaw_Speed_ki = 0.05f; //0.0
  float Yaw_Speed_kd = 1.0f; //0.0
	float Yaw_Speed_kf = 1.0f; //2.0
	float Yaw_Angle_kp = 0.15f; //0.16
	float Yaw_Angle_ki = 0.0f; //0.0
	float Yaw_Angle_kd = 0.5f; //0.0   
	float Yaw_Angle_kf = 0.5f; //2.0

/*自瞄参数*/
	float Yaw_Speed_kp_Shoot = 55.0f; //65
  float Yaw_Speed_ki_Shoot = 0.0f; //0.12
  float Yaw_Speed_kd_Shoot = 20.0f; //0.9
	float Yaw_Speed_kf_Shoot = 4.0f; //4.0
	float Yaw_Angle_kp_Shoot = 0.22f; //0.23
	float Yaw_Angle_ki_Shoot = 0.0f; //0.0
	float Yaw_Angle_kd_Shoot = 12.0f; //2.5   
	float Yaw_Angle_kf_Shoot = 4.5f; //4.5 


//限幅
	float Pitch_Angle_PLimit = 99999.0f;  /*< 角度环P比例误差限幅 */
	float Pitch_Angle_ILimit = 99999.0f;  /*< 角度环I积分误差限幅 */
	float Pitch_Speed_PLimit = 99999.0f;  /*< 速度环P比例误差限幅 */
	float Pitch_Speed_ILimit = 99999.0f;  /*< 速度环I积分误差限幅 */

	float Yaw_Angle_Shoot_PLimit = 99999.0f;
	float Yaw_Angle_PLimit = 49999.0f;  //34999.0 /*< 角度环P比例误差限幅 */
	float Yaw_Angle_ILimit = 99999.0f;  /*< 角度环I积分误差限幅 */
	float Yaw_Speed_Shoot_PLimit = 99999.0f;
	float Yaw_Speed_PLimit = 49999.0f;  //49999.0 /*< 速度环P比例误差限幅 */
	float Yaw_Speed_ILimit = 99999.0f;  /*< 速度环I积分误差限幅 */
	
	float holder_pidlim = 30000; /* pid输出限幅：30000 */

//巡航扫描范围
int32_t Pitch_Cruise_Min[2] = {170000, 170000};//172000, 182500  
int32_t Pitch_Cruise_Max[2] = {228000, 228000};//193500, 185500
int32_t Pitch_Delta[2] = {50, 120};
int32_t Yaw_Cruise_Max = 50000;
int32_t Yaw_Cruise_Min = -50000;
//遥控器、视觉范围
int32_t Pitch_Min = 170000;//197200
int32_t Pitch_Max = 228000;//160000
int32_t Pitch_Fix_Angle = 195000;
int32_t Pitch_Outpost = 195000;
//1348 1837 
int32_t Yaw_Delta_Max = 500;//100;
int32_t Yaw_Delta_Min = 500;//800;

float gain_f = 573.0;
#endif /*< ifdef HOLDER_CONFIG_H*/


