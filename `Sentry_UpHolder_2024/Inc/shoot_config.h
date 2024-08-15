#ifndef _SHOOT_CONFIG_H
#define _SHOOT_CONFIG_H
#include <stdint.h>
#include "config.h"
#include <stdbool.h>
/******************************************************************************
  * @brief  发射机构控制相关参数
  * @attention 每次只能定义一个波形参数，调试完务必注释掉debug相关定义
  *     1.拨弹电机2006单环绝对式PID控制
  *     速度环）采用绝对式PID控制器
  *     调试方法：
  *         1.定义 #define SHOOT_FREQ_DEBUG，射频控制，可通过debug在线修改参数
  *         2.定义 #define SHOOT_FREQ_WAVE，射频控制，可输出拨弹电机波形
  *         3.定义 #define SHOOT_SPEED_DEBUG, 射速控制
  *         4.定义 #define SHOOT_SPEED_WAVE, 射速波形
  *         具体实现方法在shoot.c中的static void Shoot_PidRun(void)函数中
  *
  *     2.摩擦轮电机snail开环控制
  ****************************************************************************/
	
/* define --------------------------------------------------------------------*/	

/* variables -----------------------------------------------------------------*/
uint16_t Shoot_heat = 0;		/*< 枪口实时热量*/
int16_t shoot_rate = 10;	/*< 射频*/
int16_t shoot_freq = 45;  /* 遥控状态自动给定，自动状态需根据局势判断 */
bool Shoot_cool = false;  /*< 枪口热量正在冷却的标志位*/      
bool Shoot_single = false;

/* 拨弹PID参数 */  
	float Shoot_Speed_kp = 4.0;//5
	float Shoot_Speed_ki = 0.04;
	float Shoot_Speed_kd = 12.0; //0.3
	float Shoot_Speed_kf = 0.0;
	float Shoot_Angle_kp = 800;
	float Shoot_Angle_ki = 0;
	float Shoot_Angle_kd = 2500;    //600    
	float Shoot_Angle_kf = 0.0;

/* 摩擦轮PID参数 */
	float Shoot_3508_Speed_kp = 35.0;//5
	float Shoot_3508_Speed_ki = 0.0;
	float Shoot_3508_Speed_kd = 0.0; //0.3
	float Shoot_3508_Speed_kf = 0.0;
/*摩擦轮限速参数*/
//夏天参数
int16_t Shoot_Speed_Max_M3508_Lim15 = 4500;
int16_t Shoot_Speed_Max_M3508_Lim18 = 4950;
int16_t Shoot_Speed_Max_M3508_Lim22 = 5550;
int16_t Shoot_Speed_Max_M3508_Lim30 = 7000;

//限幅
float shoot_kp_limit = 30000;
float shoot_ki_limit = 30000;
float shoot_pidlim = 30000;
float Shoot_Freq_out_Limit= 10000;
float shoot_2006_output_limit = 10000;

#endif

