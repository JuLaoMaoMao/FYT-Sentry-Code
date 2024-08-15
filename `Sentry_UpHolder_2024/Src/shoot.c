/**
  ******************************************************************************
  * @file    
  * @author  zz
  * @brief
  * @date     
  ******************************************************************************
  * @attention
  *
  * Copyright (c) CSU_RM_FYT.
  * All rights reserved.
  *
  * This software component is licensed by SY under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* includes ------------------------------------------------------------------*/
#include "assist_fun.h"
#include "can.h"
#include "Holder.h"
#include "Judge.h"
#include "Judge_Tx.h"
#include "pid.h"
#include "shoot.h"
#include "shoot_config.h"
#include "tim.h"
#include "usart.h"
#include "Vision.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
float one_angle = 5.143;
int32_t shoot_heat_cnt,shoot_cal_heat;
/* 
    减速比36：1，拨盘7个拨爪，电机每转36/7 = 5.143圈拨一个弹丸
*/

SHOOT_T Shoot_M2006 = {0};
SHOOT_3508_T Shoot_M3508[2] = {0};
/* function ------------------------------------------------------------------*/
/**
  * @brief  拨盘PID初始化
  * @para   void
  * @retval void
  * @attention 
  */
void Shoot_PIDInit(void)
{ 
		/*2006*/
		pid_init_absolute(&Shoot_M2006.PidAbs_Position, Shoot_Angle_kp, Shoot_Angle_ki, Shoot_Angle_kd,Shoot_Angle_kf,shoot_kp_limit, shoot_ki_limit, -shoot_pidlim, shoot_pidlim); 
		pid_init_absolute(&Shoot_M2006.PidAbs_Speed, Shoot_Speed_kp, Shoot_Speed_ki, Shoot_Speed_kd, Shoot_Speed_kf,shoot_kp_limit, shoot_ki_limit, -shoot_pidlim, shoot_pidlim);    
		Shoot_M2006.TarAngle = Shoot_M2006.Angle;
		
		/*3508*/
		pid_init_absolute(&Shoot_M3508[0].PidAbs_Speed, Shoot_3508_Speed_kp, Shoot_3508_Speed_ki, Shoot_3508_Speed_kd, Shoot_3508_Speed_kf,shoot_kp_limit, shoot_ki_limit, -shoot_pidlim, shoot_pidlim); 
		pid_init_absolute(&Shoot_M3508[1].PidAbs_Speed, Shoot_3508_Speed_kp, Shoot_3508_Speed_ki, Shoot_3508_Speed_kd, Shoot_3508_Speed_kf,shoot_kp_limit, shoot_ki_limit, -shoot_pidlim, shoot_pidlim);
}

/**
  * @brief  PID计算
  * @param  void
  * @retval void
  * @attention 
  */
void Shoot_PidCalculation(REMOTE_DATA_T RemoteMsg)
{
	  /* 滤波处理 */
	 	static int32_t RMF_Speed_Buf[4];  /* 速度反馈四点加权滤波缓冲区*/
		Shoot_M2006.RmfSpeed = Misc_s32Recursive_Mean4_Filter(Shoot_M2006.Speed, RMF_Speed_Buf);  
	
		Shoot_M2006.TarSpeed = pid_absolute_update(Shoot_M2006.TarAngle, Shoot_M2006.Angle, &Shoot_M2006.PidAbs_Position);
		Shoot_M2006.Output_Lpf = pid_absolute_update(Shoot_M2006.TarSpeed,Shoot_M2006.RmfSpeed,&Shoot_M2006.PidAbs_Speed);
		Shoot_M2006.Output = Shoot_M2006.Output_Lpf * 0.7 + Shoot_M2006.Output * 0.3;
		Shoot_M2006.Output = Constrain_Int32_t(Shoot_M2006.Output, -Shoot_Freq_out_Limit, Shoot_Freq_out_Limit);     //输出限幅 

		Shoot_M3508[0].Output = pid_absolute_update(Shoot_M3508[0].TarSpeed,Shoot_M3508[0].Speed,&Shoot_M3508[0].PidAbs_Speed);
		Shoot_M3508[1].Output = pid_absolute_update(Shoot_M3508[1].TarSpeed,Shoot_M3508[1].Speed,&Shoot_M3508[1].PidAbs_Speed);
}

/**
  * @brief  遥控
  * @param  控制指令结构体
  * @retval void
  * @attention
  */
void Shoot_Manual(REMOTE_DATA_T RemoteMsg)
{			
	  /* 单发模式 */
		if(shoot_flag == 1)
		{
			 if(Shoot_single == false)
			 {
			    Shoot_M2006.TarAngle -= one_angle;	
					shoot_heat_cnt++;
          Shoot_single = true;				 
			 }
		}
		else if(shoot_flag == 0)
		{
//		   Shoot_Reset();
			 Shoot_single = false;
		}
		
		/* 模拟自动开火 */
		else if(shoot_flag == 2 )//&& vData.recv_data.firesuggestion == 0x01)
		{
			shoot_freq--;
			if (shoot_freq <= 0 && Shoot_cool == false)
			{
					Shoot_M2006.TarAngle -= 0.5f * one_angle;
					shoot_freq = 25;
//					shoot_heat_cnt++;
//					Shoot_HeatControl();  /* 手动给值 */
					if(Misc_Fabsf(Shoot_M2006.TarAngle - Shoot_M2006.Angle) >= 25) /* 连续五发以上堵转 */
					{
							Shoot_M2006.TarAngle = Shoot_M2006.Angle + one_angle; /* 往回退一格 */
					}
			}
		}

}

/**
  * @brief  自动
  * @param  
  * @retval void
  * @attention 
  */
void Shoot_Auto(REMOTE_DATA_T RDMsg)
{
//	 #ifdef SHOOT_TEST  
//			shoot_freq--;
//			if (shoot_freq <= 0)
//			{
//					Shoot_M2006.TarAngle -= 0.5f * one_angle;
//					Shoot_GetRate();
//			} 

//	 #else
//		static bool shoot_stop_flag;
		static int16_t shoot_cnt;
		if (JUDGE_u8game_progress() == 4 || Remote_Data.S2 == 1)
		{
					if (vData_z > 0.1f && vData_z <= 10.0f )/* 已锁定*/
					{
							shoot_cnt = 100; /* 丢失目标后计数 */
					}
					else
					{
							shoot_cnt --;
							if(shoot_cnt <= 0)	
								 shoot_cnt = 0;
					}
						
					/* 拨弹开火条件判断 */
//					if(vData.recv_data.firesuggestion == 0x01 && JUDGE_u16bullet_remaining_num_17mm() > 10)
//					{
							if (shoot_cnt > 0 && Shoot_cool == false && JUDGE_u16bullet_remaining_num_17mm() > 10)
							{
									shoot_freq--;
									if (shoot_freq <= 0 && vData.recv_data.firesuggestion == 0x01)
									{
											Shoot_M2006.TarAngle -= 0.5f * one_angle;
											shoot_heat_cnt++;
											if(vData_z <= 6.0f)
												shoot_freq = 35;
											else 
												shoot_freq = 50;
									}
									
									/* 防堵转 */
									if(Misc_Fabsf(Shoot_M2006.TarAngle - Shoot_M2006.Angle) >= 25) /* 连续五发以上堵转 */
									{
											Shoot_M2006.TarAngle = Shoot_M2006.Angle + one_angle; /* 往回退一格 */
//											shoot_stop_flag = true;
									}
//									if(shoot_stop_flag == true && (Misc_Fabs16(Shoot_M2006.TarSpeed - Shoot_M2006.RmfSpeed) >= 400))
//									{	
//											Shoot_Reset();
//									}
//									else
//											shoot_stop_flag = false;
							}
//							else
//									Shoot_Reset();
//					}
		}
		else
			 Shoot_Reset();
//	#endif	
}

/**
  * @brief  枪管热量获取
  * @param  void
  * @retval void
  * @attention
  */
void Shoot_GetHeat(void)
{		
		if(JUDGE_u16GetRemoteHeat17(1) == 0)
				Shoot_heat = shoot_cal_heat;
		else
				Shoot_heat = JUDGE_u16GetRemoteHeat17(1);
}

/**
  * @brief  射频换算与热量控制
  * @param  void
  * @retval void
* @attention 每隔 shoot_freq*2ms 拨一颗弹丸，故射频为 500/shoot_freq 个/s
  */
void Shoot_HeatControl(void)
{
		/* 热量控制*/	
		if (Shoot_heat > 380 || Shoot_cool == true)
		{
				Shoot_cool = true;
		}
		if (Shoot_heat < 20)
				Shoot_cool = false;
}
/*重要！射频需自行测试，尽量取快，保证又快又准*/
/**
  * @brief  拨弹模式选择
  * @param  void
  * @retval void
  * @attention 
  */
void Shoot_ChooseMode(REMOTE_DATA_T RemoteMsg)
{
   if(Remote_Data.S1 == 1 && Remote_Data.S2 == 1)    /* 遥控模式 */
		 Shoot_Manual(RemoteMsg);
	 else if(Remote_Data.S1 == 2)
	 {
//		 	if(RemoteMsg.S2 == 3)
//				Shoot_Manual(RemoteMsg);
//			else
				Shoot_Auto(RemoteMsg);	
	 }
	 else if(Remote_Data.S1 == 3 && Remote_Data.S2 == 1)
		 Shoot_Manual(RemoteMsg);
	 else
		 Shoot_Reset();
}

/**
  * @brief  发送函数
  * @param  void
  * @retval void
  * @attention 
  */
void Shoot_CanTransmit(void)
{
    if(flag) //
    {
				Shoot_M2006.Can_Data[0] = (uint8_t)(Shoot_M3508[0].Output >> 8);
				Shoot_M2006.Can_Data[1] = (uint8_t)(Shoot_M3508[0].Output);
				Shoot_M2006.Can_Data[2] = (uint8_t)(Shoot_M3508[1].Output >> 8);
				Shoot_M2006.Can_Data[3] = (uint8_t)(Shoot_M3508[1].Output);
		    Shoot_M2006.Can_Data[4] = (uint8_t)(Shoot_M2006.Output >> 8);
        Shoot_M2006.Can_Data[5] = (uint8_t)(Shoot_M2006.Output);			
    }   
    else
    {
			 Shoot_Reset();
			 memset(Shoot_M2006.Can_Data, 0, sizeof(Shoot_M2006.Can_Data));
    }
		
		CAN1_Transmit(0x200, Shoot_M2006.Can_Data);
}

/**
  * @brief  总体拨弹进程
  * @param  void
  * @retval void
  * @attention 
  */
void Shoot_Process(REMOTE_DATA_T RemoteMsg)
{
//		hh1++;
		Shoot_GetHeat();
		Shoot_HeatControl();
	  Shoot_ChooseMode(RemoteMsg);
		FrictionWheel_Process(RemoteMsg);
    Shoot_PidCalculation(RemoteMsg);
}

/**
  * @brief  摩擦轮进程控制
  * @param  void
  * @retval void
  * @attention 
  */
uint16_t sbszh = 0;
uint16_t speed_limit = 0;
uint16_t shoot_m3508_speed_now[2] = {0};
void FrictionWheel_Process(REMOTE_DATA_T RemoteMsg)
{  
		 if(flag) /*< 遥控器保护，数据量16时才开启控制 */
		 { 
				 switch(Remote_Data.S1)
				 {
						case SUP: /* 遥控模式 */
								if(Remote_Data.S2 != 2)
									speed_limit = Shoot_Speed_Max_M3508_Lim30;
								else
									speed_limit = Shoot_Speed_Max_M3508_Lim30;
						break;
						
						case SDOWN: /* 比赛模式 */
								speed_limit = Shoot_Speed_Max_M3508_Lim30;
						break;

						case SMID: /* 停止 */		
								if(Remote_Data.S2 == 1)
									speed_limit = Shoot_Speed_Max_M3508_Lim30;
								else
									speed_limit = 0;
						break;
				 }
		 }
		 else
			 speed_limit = 0;
		 
		Shoot_M3508[0].TarSpeed =  -speed_limit;
    Shoot_M3508[1].TarSpeed = speed_limit;

    /* 斜坡函数，缓慢提升速度，加速过大会导致云台抖动 */
    Shoot_M3508[0].TarSpeed = RAMP_Uint16(Shoot_M3508[0].TarSpeed, shoot_m3508_speed_now[0], 4);
    Shoot_M3508[1].TarSpeed = RAMP_Uint16(Shoot_M3508[1].TarSpeed, shoot_m3508_speed_now[1], 4);

    shoot_m3508_speed_now[0] = Shoot_M3508[0].TarSpeed;
    shoot_m3508_speed_now[1] = Shoot_M3508[1].TarSpeed;
}

/**
  * @brief  拨弹停止
  * @param  void
  * @retval void
  * @attention 
  */
void Shoot_Reset(void) 
{
		Shoot_M2006.Output = 0;
    Shoot_M2006.TarAngle = Shoot_M2006.Angle;
    Shoot_M2006.TarSpeed = 0;
    Shoot_M2006.PidAbs_Speed.errOld = 0;
    Shoot_M2006.PidAbs_Speed.errP = 0;
    Shoot_M2006.PidAbs_Speed.errNow = 0;
    Shoot_M2006.PidAbs_Speed.ctrOut = 0;
    Shoot_M2006.PidAbs_Speed.errD = 0;
    Shoot_M2006.PidAbs_Speed.errI = 0;
		Shoot_M2006.PidAbs_Position.errOld = 0;
    Shoot_M2006.PidAbs_Position.errP = 0;
    Shoot_M2006.PidAbs_Position.errNow = 0;
    Shoot_M2006.PidAbs_Position.ctrOut = 0;
    Shoot_M2006.PidAbs_Position.errD = 0;
    Shoot_M2006.PidAbs_Position.errI = 0;
		Shoot_M2006.Output = 0;
	  Shoot_M2006.Output_Lpf = 0;
	  
	for(uint8_t i = 0;i <= 1;i++ )
	{
		Shoot_M3508[i].TarSpeed = 0;
		Shoot_M3508[i].Output = 0;
		Shoot_M3508[i].Output_Lpf = 0;
    Shoot_M3508[i].PidInc_Speed.errNow  = 0;
    Shoot_M3508[i].PidInc_Speed.errOld1 = 0;
    Shoot_M3508[i].PidInc_Speed.errOld2 = 0;
    Shoot_M3508[i].PidInc_Speed.ctrOut  = 0;
    Shoot_M3508[i].PidInc_Speed.dCtrOut = 0;
	}
		memset(Shoot_M2006.Can_Data, 0, sizeof(Shoot_M2006.Can_Data));
}	


/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/

