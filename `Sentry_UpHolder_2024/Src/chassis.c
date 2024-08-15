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
  ******************************************************************************
  */

/* includes ------------------------------------------------------------------*/
#include "assist_fun.h"
#include "can.h"
#include "chassis.h"
#include "chassis_config.h"
#include "config.h"
#include "FreeRTOS.h"
#include "Holder.h"
#include "Judge.h"
#include "Judge_tx.h"
#include "message.h"
#include "task.h"
#include "tim.h"
#include "usart.h"
#include "Vision.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/ 
Chassis_T Chassis = {0};
uint8_t control_state = 0x00;
static uint8_t supercap_state_tx = CHASSIS_SUPERCAP;//CHASSIS_SUPERCAP;  /* 超级电容状态 */
static float average_spin;         /* 小陀螺平均转速 */
static bool spin_ramp = true;      /* 小陀螺斜坡标志位 */
/* function ------------------------------------------------------------------*/
/**
  * @brief  获取陀螺仪的底盘数据
  * @param  控制指令结构体
  * @retval void
  * @attention 
  */	
void IMU_GetChassisData(Chassis_Data_T* ChassisMsg)
{
	Chassis.IMU_Angle = ChassisMsg->angle[2];
	Chassis.IMU_Angle_Lpf = Chassis.IMU_Angle * 0.85 + Chassis.IMU_Angle_Lpf * 0.15;
	
	Chassis.IMU_Speed = ChassisMsg->gyro[2];
	Chassis.IMU_Speed_Lpf = Chassis.IMU_Speed * 0.85 + Chassis.IMU_Speed_Lpf * 0.15;
}

/**
  * @brief  PID初始化
  * @param  void
  * @retval void
  * @attention 
  */	
void Chassis_PidInit(void)
{
	for(int i = 1;i <= 4;i++)
	{
		 pid_init_increment(&Chassis.M3508[i].PidInc_Speed, M3508_Speed_kp, M3508_Speed_ki, M3508_Speed_kd, 30000);        
		 pid_init_increment(&Chassis.M3508[i].PidInc_Current, M3508_Current_kp, M3508_Current_ki, M3508_Current_kd, 30000); 				
	}
}

/**
  * @brief  PID计算
  * @param  void
  * @retval void
  * @attention 
  */
void Chassis_PidCalculate(void)
{  
	 for(int i = 1;i <= 4;i++)
	 {
		  Chassis.M3508[i].Speed = (float)Chassis.M3508[i].Speed_Rx;
		  Chassis.M3508[i].Speed_Lpf = Chassis.M3508[i].Speed * 0.7f + Chassis.M3508[i].Speed_Lpf * 0.3f; //低通滤波
		 
		  Chassis.M3508[i].Current = (float)Chassis.M3508[i].Current_Rx;
	    Chassis.M3508[i].Current_Lpf = Chassis.M3508[i].Current * 0.7f + Chassis.M3508[i].Current_Lpf * 0.3f; //低通滤波
		 
      Chassis.M3508[i].TarCurrent = pid_increment_update(Chassis.M3508[i].TarSpeed, Chassis.M3508[i].Speed_Lpf, &Chassis.M3508[i].PidInc_Speed);
		  Chassis.M3508[i].Output = pid_increment_update(Chassis.M3508[i].TarCurrent, Chassis.M3508[i].Current_Lpf, &Chassis.M3508[i].PidInc_Current);
		 
	  /* 输出限幅 */
    Chassis.M3508[i].Output = constrain_int16_t(Chassis.M3508[i].Output, -Chassis_Out_Limit, Chassis_Out_Limit);
	 }
}

/**
  * @brief  获取遥控器移动数据
  * @param  遥控器消息结构体指针
  * @retval void
  * @attention
			Ch2控制左右：Chassis.MoveData.Left；
			Ch3控制前后：Chassis.MoveData.Front；
			Ch0控制旋转：Chassis.MoveData.ClockWise；
  */
void GetRemoteMoveData(REMOTE_DATA_T RemoteMsg)
{
    Chassis.MoveData.Front = (float)(RemoteMsg.Ch3)/300.0f;
    Chassis.MoveData.Left = (float)(-RemoteMsg.Ch2)/300.0f;
}

float Chassis_StateControl(void)
{
     float spin_temp;
		 Chassis.MoveData.Front = vData_x_speed;
		 Chassis.MoveData.Left = vData_y_speed;					
		 spin_temp = vData_yaw_speed; 
   
     return spin_temp;	
}


/**
  * @brief  跟随云台模式
  * @param  遥控器消息结构体
  * @retval 
  * @attention 
  */
float Follow_Holder(float dir_angle)  
{
    float tmp_spin = 0;
	  int8_t sign_speed = 0;
	  
	  sign_speed = dir_angle > 0? 1:-1;
	
	  if(Misc_Fabsf(dir_angle) <= 4)
			tmp_spin = 0;
		else
      tmp_spin = constrain_float(sign_speed * dir_angle * dir_angle * 0.004f, -1.25f*PI1, 1.25f*PI1); 

    return tmp_spin;
}

/**
  * @brief  小陀螺
  * @param  void
  * @retval void
  * @attention 斜坡上升小陀螺
  */
float DynamicSpin(void) 
{
	  static float spin_tmp;
	  if(vData.recv_data.isNavigating == 0x01)	spin_ramp = true;		
	
	  if(spin_ramp == true)
		{
		    spin_tmp = RAMP_float(average_spin, spin_tmp, 0.05f); /* 斜坡函数，使转速缓慢上升 */
			  if(spin_tmp >= average_spin) 
				{
				   spin_tmp = average_spin;
					 spin_ramp = false;
				}
		}
		return spin_tmp;					
}

/**
  * @brief  速度微调
  * @param  void
  * @retval void
  * @attention 云台手按键操控
  */
float Speed_tunning(void) 
{
	  float spin_temp;
	  switch(JUDGE_u8_robot_command())
		{
		  case 65:
          Chassis.MoveData.Front = 0.0f;			
				  Chassis.MoveData.Left = 0.1f;	
			    spin_temp = 0.0f;	
			break;
			
			case 68:
				  Chassis.MoveData.Front = 0.0f;	
				  Chassis.MoveData.Left = -0.1f;
			    spin_temp = 0.0f;
			break;

			case 83:
				  Chassis.MoveData.Front = -0.1f;	
				  Chassis.MoveData.Left = 0.0f;
			    spin_temp = 0.0f;			
			break;
			
			case 87:
				  Chassis.MoveData.Front = 0.1f;
			    Chassis.MoveData.Left = 0.0f;
			    spin_temp = 0.0f;
      break;

			case 81:
				  Chassis.MoveData.Front = 0.0f;
			    Chassis.MoveData.Left = 0.0f;
			    spin_temp = 0.6f;
      break;

			case 69:
				  Chassis.MoveData.Front = 0.0f;
			    Chassis.MoveData.Left = 0.0f;
			    spin_temp = -0.6f;
      break;

			case 70:
				  Chassis.MoveData.Front = 0.0f;
			    Chassis.MoveData.Left = 0.0f;
			    spin_temp = 0.0f;
      break;
			
      default:
          Chassis.MoveData.Front = 0.0f;
          Chassis.MoveData.Left = 0.0f;	
			    spin_temp = 0.0f;
      break;			
		}
		
		return spin_temp;
}

/**
  * @brief ROS控制模式
  * @param  
  * @retval 
  * @attention 
  */
float RosControl(void)
{
	  float spin_tmp;
//    if (JUDGE_u8game_progress() == 4)    //正式比赛开始
//	  {
//				switch(JUDGE_u8_robot_command())
//				{
//					case 'R': control_state = 0x00; break;
//					case 'Z': control_state = 0x01; break;
//					case 'X': control_state = 0x02; break;
//					case 'V': control_state = 0x03; break;
//					case 'C': control_state = 0x04; break;
//					case 82: control_state = 0x00; break;
//					case 90: control_state = 0x01; break;
//					case 88: control_state = 0x02; break;
//					case 86: control_state = 0x03; break;
//					case 67: control_state = 0x04; break;
//					default: break;
//				}
				
				if(vData.recv_data.isNavigating == 0x01)
				{
					 move_state = ChassisMode;
					 Chassis.MoveData.Front = vData_x_speed;
					 Chassis.MoveData.Left = vData_y_speed;					
				   spin_tmp = -vData_yaw_speed;
				}
				else if(vData.recv_data.isNavigating == 0x00 && vData.recv_data.isSpining == 0x01)
				{
					 move_state = ChassisMode;
					 Chassis.MoveData.Front = 0.0f;
					 Chassis.MoveData.Left = 0.0f;	
//					 if(Outpost_HP <= 10) spin_tmp = DynamicSpin(); 
//           else spin_tmp = 0;      
           spin_tmp = DynamicSpin();					
				} 
				else
				{
//					 move_state = HolderMode;
//				   spin_tmp = Speed_tunning();
           move_state = ChassisMode;					
					 Chassis.MoveData.Front = 0.0f;
					 Chassis.MoveData.Left = 0.0f;
           spin_tmp = 0;					
				}
//		}		
//		else
//		{
//				 move_state = ChassisMode;
////			   control_state = 0x00;
//				 Chassis.MoveData.Front = 0;
//				 Chassis.MoveData.Front = 0;
//				 spin_tmp = 0;
//		}
		
		return spin_tmp;
}

/**
  * @brief  底盘模式选择
  * @param  void
  * @retval void
  * @attention S1往上为遥控器控制，S1往下为小陀螺/Ros状态，中间为停止
  */
void Chassis_ChooseMode(REMOTE_DATA_T RemoteMsg)
{
			float spin_tmp = 0; 
			float dir_angle = GetDirAngle_Shoot();
		
			switch(RemoteMsg.S1)
			{
					case SUP:
						bias_angle_shoot = -1000;
						GetRemoteMoveData(RemoteMsg);
					  move_state = ChassisMode;
						switch(RemoteMsg.S2)
							{
								case SUP:   /* 跟随云台 */
//										spin_tmp = Follow_Holder(dir_angle); 
										spin_tmp = -RemoteMsg.Ch0/100;
								break;
															
								case SMID:  /* 底盘停止 */
										Chassis_Protect();
								break;
								
								case SDOWN: /* 跟随遥控器 */
										spin_tmp = RemoteMsg.Ch0/200;
								break;
						}
					break; 
							
					case SDOWN:
						bias_angle_shoot = -3070;
						switch(RemoteMsg.S2)
							{
								case SUP: /* 比赛备用模式：定点模式小陀螺+自瞄 */
										 move_state = ChassisMode;
										 if(JUDGE_u8game_progress() == 4)
										 {
												Chassis.MoveData.Front = 0;
												Chassis.MoveData.Left = 0;
//											  if(Outpost_HP <= 10)
											    spin_tmp = DynamicSpin(); 	
//                        else
//                         	spin_tmp = 0;	
//											    spin_tmp = DynamicSpin(); 	
//                         	spin_tmp = 0;	
									
										 }
										 else
										 {
											 Chassis.MoveData.Front = 0;
											 Chassis.MoveData.Left = 0;
										   spin_tmp = 0;
										 }
								break;
								
								case SMID: /* 遥控模式小陀螺 */
										 move_state = HolderMode;	  
                     GetRemoteMoveData(RemoteMsg);								
										 spin_tmp = DynamicSpin();
								break;
								
								case SDOWN: /* ROS控制*/
										 spin_tmp = RosControl();
								break;
							}           
				 break; 
							
				 case SMID: /* 底盘停止 */
							bias_angle_shoot = -3070;
						  Chassis_Protect();
							if(RemoteMsg.S2 != 3) /* 手动切换超级电容模式 */
							{
							   supercap_state_tx = CHASSIS_POWER;
							}
							else
							{
							   supercap_state_tx = CHASSIS_SUPERCAP;
							}								
				 break; 			
			}
			
			/* 输出限幅 */	
			Chassis.MoveData.ClockWise = constrain_float(spin_tmp, -3*PI1, 3*PI1);	
}

/**
  * @brief  超级电容底盘功率控制
  * @param  void
  * @retval void
  * @attention  
  */
void Chassis_PowerControl(void)
{
			/* 发送超级电容控制数据 */
			SuperCapCanTransmit();
	
      /* 电压足够高、功率上限前开启超级电容 */
				if(Chassis.SuperCap.cap_vol >= 14000 && (supercap_state_tx == CHASSIS_SUPERCAP || Chassis.SuperCap.sup_state == 0))
				{
					 supercap_state_tx = CHASSIS_SUPERCAP;
						 
					/* 根据电容电压设置小陀螺转速 */
					 if(Chassis.SuperCap.cap_vol >= 20000)
					 {
							average_spin = 1.45f * PI1;//1.48
					 } 						  					 
					 else if(Chassis.SuperCap.cap_vol >= 16000)
					 {
							average_spin = 1.38f * PI1;//1.38
					 }								 
					 else 
					 {
							Chassis.MoveData.Front = Chassis.MoveData.Front/1.35f;
							Chassis.MoveData.Left = Chassis.MoveData.Left/1.35f;
							average_spin = 1.3f * PI1;//1.3
					 }					
				 }
			else  /* 电压过低则关闭电容 */
			{							
				  Chassis.MoveData.Front = Chassis.MoveData.Front/2.0f;
				  Chassis.MoveData.Left = Chassis.MoveData.Left/2.0f;
					average_spin = 1.25f * PI1;		  //1.25
				
					/* 电容会自动充电，电压足够高时切换回电容模式 */
					if(Chassis.SuperCap.cap_vol >= 20000 && supercap_state_tx == CHASSIS_POWER)
						 supercap_state_tx = CHASSIS_SUPERCAP;  
					else
						 supercap_state_tx = CHASSIS_POWER;
			}				
}

/**
  * @brief  超级电容发送
  * @param  void
  * @retval void
  * @attention 
  */
uint16_t power_limit_tx;
static void SuperCapCanTransmit(void)
{
		 power_limit_tx = JUDGE_u16GetChassisPowerLimit();
	   power_limit_tx = Constrain_Uint16_t(power_limit_tx, 20, 150);
		 Chassis.SuperCap.can_data[0] = (uint8_t)power_limit_tx;
		 Chassis.SuperCap.can_data[1] = supercap_state_tx;
		 Chassis.SuperCap.can_data[2] = Chassis.SuperCap.can_data[0] + 10;
		 CAN2_Transmit(0x210, Chassis.SuperCap.can_data);
}

/**
  * @brief  底盘运动学解算
  * @param  void
  * @retval void
  * @attention 
  */
void Chassis_Kinematic(void)
{
			float tmp_wheel_speed[4];
			float dir_angle = (GetDirAngle_Shoot() * PI1/180.0f);//rad
			static float front, left;  
			
			switch(move_state)
			{
					case HolderMode: 
						front_temp = Chassis.MoveData.Front * cosf(dir_angle) - Chassis.MoveData.Left * sinf(dir_angle); 
						left_temp = Chassis.MoveData.Front * sinf(dir_angle) + Chassis.MoveData.Left * cosf(dir_angle);
					break;
								
					case ChassisMode: 
						 front_temp = Chassis.MoveData.Front;
						 left_temp =	Chassis.MoveData.Left; 
					break; 
					
					default: 
						 front_temp = Chassis.MoveData.Front;
						 left_temp = Chassis.MoveData.Left;	           
					break;
			}			

			front = front_temp;
		  clockwise = Chassis.MoveData.ClockWise;			
			if(vData.recv_data.isSpining == 0x01 && vData.recv_data.isNavigating == 0x00)
			{
				 front = RAMP_float(front_temp, front, 0.01f);
			   left = RAMP_float(left_temp, left, 0.01f);			
			}
      else
         left = left_temp;								
		
			/* 底盘逆向轮速解算 *//*规定x向前，y向左，逆时针为正*/
			tmp_wheel_speed[0] =  (-front/sqrt(2) - left/sqrt(2) - clockwise * Chassis_r)/GAIN_F;  
			tmp_wheel_speed[1] = -(-front/sqrt(2) + left/sqrt(2) + clockwise * Chassis_r)/GAIN_F;
			tmp_wheel_speed[2] =  (-front/sqrt(2) + left/sqrt(2) - clockwise * Chassis_r)/GAIN_F;
			tmp_wheel_speed[3] = -(-front/sqrt(2) - left/sqrt(2) + clockwise * Chassis_r)/GAIN_F;
			/*全向轮解算*/
			/*1 0
			  3 2*/
			
			for(int j = 0; j <= 3; j++)
				 Chassis.M3508[j + 1].TarSpeed = tmp_wheel_speed[j];
//			WheelSpeedConstrait(tmp_wheel_speed); /* 速度限幅 */
}

/**
  * @brief  底盘轮速限制
  * @param  void
  * @retval void
  * @attention 
  *    如果出现某个轮速大于最大值或小于最小值的情况。
  *    将该轮速缩放到限制速度以内，其他3个轮速也要相同比例缩放。（该函数可能拖延进程，看情况而定）
  */
void WheelSpeedConstrait(float* tmp_wheel_speed)
{
		float limit_speed = fmax_wheel_speed;
		float tmp_max_speed = 1.0f;
		float tmp_min_speed = 1.0f;
		float cmp_index = 1.0f;

		for(int i = 0; i <= 3; i++)
		{   
				if (tmp_wheel_speed[i] > tmp_max_speed) tmp_max_speed = tmp_wheel_speed[i];  /* 找最大值 */
				if (tmp_wheel_speed[i] < tmp_min_speed) tmp_min_speed = tmp_wheel_speed[i];  /* 找最小值 */
		}

		tmp_max_speed = Misc_Fabsf(tmp_max_speed);
		tmp_min_speed = Misc_Fabsf(tmp_min_speed);

		if (tmp_max_speed > limit_speed || tmp_min_speed > limit_speed)
		{
				if (tmp_max_speed <= tmp_min_speed) 
					 cmp_index = limit_speed/tmp_min_speed;
				
				else if (tmp_max_speed > tmp_min_speed) 
					 cmp_index = limit_speed/tmp_max_speed;
		}
		else
				cmp_index = 1;
	 
			for(int j = 0; j <= 3; j++)
				 Chassis.M3508[j + 1].TarSpeed = (tmp_wheel_speed[j] * cmp_index);
}

/**
* @brief  CAN发送
  * @param  void
  * @retval void
  * @attention 放中断里面
  */
void Chassis_CanTransmit(void)
{
    if (Observer.Tx.DR16_Rate > 15) /* 遥控器保护，数据量16时才开启控制 */
    {     
       for(int j = 0,k = -1;j <= 6; j = j + 2)
	      {
					/* CAN 赋值 */
					Chassis.Can_Data[j]   = (uint8_t) (Chassis.M3508[j-k].Output >> 8);  
					Chassis.Can_Data[j+1] = (uint8_t)  Chassis.M3508[j-k].Output;
					k++;			
		    }      
    }	 
    else	
			 Chassis_Protect();

    CAN2_Transmit(0x200,Chassis.Can_Data);
}

/**
  * @brief  底盘进程
  * @param  void
  * @retval void
  * @attention 
  */
void Chassis_Process_Tmp(REMOTE_DATA_T RemoteMsg,Chassis_Data_T ChassisMsg)
{
		IMU_GetChassisData(&ChassisMsg);
		Chassis_ChooseMode(RemoteMsg);
		Chassis_PowerControl();
		Chassis_Kinematic();
		Chassis_PidCalculate();
	  Chassis_CanTransmit();
}

void Chassis_Process(REMOTE_DATA_T RemoteMsg,Chassis_Data_T ChassisMsg)
{
//	  /* 底盘模式切换后重置0.2s */
//	  if(vData.recv_data.isNavigating == 0x01) 
//		{
//			 chassis_shift_cnt = 0;
       Chassis_Process_Tmp(RemoteMsg,ChassisMsg);
//		}
//		else
//		{
//				chassis_shift_cnt ++;
//				if(chassis_shift_cnt >= 200)
//				{
//					chassis_shift_cnt = 200;
//          Chassis_Process_Tmp(RemoteMsg,ChassisMsg);					
//				}			       
//				else
//				{
//          Chassis.MoveData.Front = 0;
//					Chassis.MoveData.Left = 0;
//          Chassis.MoveData.ClockWise = 0;
//			    Chassis_Kinematic();
//			    Chassis_PidCalculate();		
//				}
//		}
}

/**
  * @brief  底盘遥控器保护
  * @param  void
  * @retval void
  * @attention 
  */
void Chassis_Protect(void)
{
	for(uint8_t i = 1;i <= 4;i++ )
	{
		Chassis.M3508[i].TarSpeed = 0;
		Chassis.M3508[i].TarCurrent = 0;
		Chassis.M3508[i].TarCurrent_Lpf = 0;
		Chassis.M3508[i].Output = 0;
		Chassis.M3508[i].Output_Lpf = 0;
    Chassis.M3508[i].PidInc_Speed.errNow  = 0;
    Chassis.M3508[i].PidInc_Speed.errOld1 = 0;
    Chassis.M3508[i].PidInc_Speed.errOld2 = 0;
    Chassis.M3508[i].PidInc_Speed.ctrOut  = 0;
    Chassis.M3508[i].PidInc_Speed.dCtrOut = 0;
    Chassis.M3508[i].PidInc_Current.errNow = 0;
    Chassis.M3508[i].PidInc_Current.errOld1 = 0;
    Chassis.M3508[i].PidInc_Current.errOld2 = 0;
    Chassis.M3508[i].PidInc_Current.dCtrOut = 0;
    Chassis.M3508[i].PidInc_Current.ctrOut  = 0;
	}  
	  Chassis.MoveData.Front = 0;
	  Chassis.MoveData.Left = 0;
	  Chassis.MoveData.ClockWise = 0;
    memset(Chassis.Can_Data, 0, sizeof(Chassis.Can_Data));
}

/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
