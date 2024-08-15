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
#include "usbd_cdc_if.h"
#include "assist_fun.h"
#include "Vision.h"
#include "config.h"
#include "can.h"
#include "string.h"
#include "Judge.h"
#include "cmsis_os.h"
#include "chassis.h"
#include "math.h"
#include "Holder.h"
#include "message.h"
#include "ins_task.h"
#include "tim.h"
//#include "usbd_cdc_if.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
#define CAMERA_X_DEVIATION 0
#define CAMERA_Y_DEVIATION 0
#define CAMERA_Z_DEVIATION 0.1f
#define CAMERA_X_DEVIATION_down 0.05f
#define CAMERA_Y_DEVIATION_down 0
#define CAMERA_Z_DEVIATION_down 0.03f
/* variables -----------------------------------------------------------------*/
vData_t vData;
CHASSIS_SPEED_T ChassisSpeed = {0};

uint8_t spining_flag = 0;
uint8_t game_state = 0x00,shoot_state = 0x00;   /* 比赛状态/自瞄标志位 */
float PI1 = 3.1415926f;
float GAIN_tran = 0.00041437; /* GAIN_F/cos45°*/
float GAIN_spin = 0.0002928;  /* GAIN_F/cos4.33°*/
float Pitch_IMU_Angle = 0,Vision_Pitch_IMU_Angle = 0;    /* 俯仰角度 */
float Yaw_IMU_Angle = 0,Vision_Yaw_IMU_Angle = 0;      /* 偏航角度 */ 
uint8_t Robot_ID = 0;					/*机器人ID*/
float Chassis_IMU_Angle = 0;  /*底盘偏航角*/
uint16_t Sentry_HP = 0;          /* 哨兵血量 */
uint16_t Remaining_Time = 0;     /* 剩余比赛时间 */
uint16_t Remaining_Bullet = 0;   /* 剩余弹量 */
uint16_t Available_Exchange_Bullet = 0, Exchanged_Bullet = 0, Total_Exchange_Bullet= 0;	/*剩余可兑换发弹量*/
uint16_t Departure_Time = 0, Remaining_Spread_Time = 0, Available_Departurn_Time = 0;
uint16_t Outpost_HP = 0, Enemy_Outpost_HP = 0;         /* 前哨战血量 */
uint8_t Hurt_ID = 0;
uint8_t IntakeArea_ID = 0, PatrolArea_ID = 0;
float vData_x, vData_y, vData_z;
float vData_x_speed, vData_y_speed, vData_yaw_speed;
uint16_t vision_protection = 0;
uint16_t vision_constant[4] = {92,95,105,0};	/*小陀螺常数，经实验测得*/
M_CIRCLE_T v_circle_yaw = {0};
float vision_yaw_angle;
uint8_t Holder_Command_ID = '0';
uint8_t test_mode = 1;
/* 卡尔曼滤波用 */
float current_yaw,last_yaw;

/* function ------------------------------------------------------------------*/
/**
  * @brief  云台接收视觉数据（USB版）
  * @param  void
  * @retval void
  * @attention 1.发送当前的底盘速度与云台角度
               2.底盘正向速度解算：
                 已知底盘四个电机的转速rmp，求解底盘的速度。向前为x轴，向左为y轴，逆时针旋转为正 
  */
void Recieve_Vision_Data(uint8_t *Recieve_Buf,uint16_t Len)
{
		/*船新通信协议*/
		vision_protection = 0;
		for(int i=0;i<Len;i++)
		{
				vData.buf[vData.index] = *(Recieve_Buf+i);
				if (vData.buf[vData.index] == 0x0D) //判断是否识别到帧尾
				{
						if (vData.buf[vData.index - 31] == 0xFF) //判断帧头是否正确
						{		
								vData.recv_data.isNavigating = vData.buf[vData.index - 28];
								vData.recv_data.isSpining_Vision = vData.buf[vData.index - 29];
								vData.recv_data.firesuggestion = vData.buf[vData.index - 30];
								memcpy(&vData.recv_data, &vData.buf[vData.index - 27], 24);
								vData.index = 0;
						}
				}
//				Vision_Pitch_IMU_Angle = -(mc_imu_pitch.Angle - dev_imu_pitch);
//				
//				Vision_Yaw_IMU_Angle = mc_imu_yaw.Angle - 1800.0f;
				
//				vData_x = Misc_Fabsf( Vision_Yaw_IMU_Angle - vData.recv_data.Yaw_Angle ) > Misc_Fabsf( Vision_Yaw_IMU_Angle + vData.recv_data.Yaw_Angle ) ?
//				-Misc_Fabsf( Vision_Yaw_IMU_Angle - vData.recv_data.Yaw_Angle ) : Misc_Fabsf( Vision_Yaw_IMU_Angle + vData.recv_data.Yaw_Angle );
				vData_x = vData.recv_data.Yaw_Angle;
				vData_y = vData.recv_data.Pitch_Angle;
				vData_z = vData.recv_data.Distance ;
				vData_x_speed = vData.recv_data.speed_x;
				vData_y_speed = vData.recv_data.speed_y;
				vData_yaw_speed = vData.recv_data.speed_yaw;
				
				if(vData_x_speed != 0 || vData_y_speed != 0 || vData_yaw_speed != 0)
					vData.recv_data.isNavigating = 1;
				else
					vData.recv_data.isNavigating = 0;
				vData.index++;
				
				if(vData_z == -1)
				{
					v_circle_yaw.Angle = mc_imu_yaw.Angle;
					v_circle_yaw.Circle = mc_imu_yaw.Circle;
				}
				/*对自瞄角度作角度连续化处理，避免零点突变*/
				if(vData.index == 200)
						vData.index = 0;
				CircleContinue(&v_circle_yaw, (vData_x + 180.0f) * 10.0f, 1800);
				vision_yaw_angle = v_circle_yaw.Angle + v_circle_yaw.Circle * 3600;
				/*视觉角度换算为目标角度*/
				gimbal_vision_pitch_tar_anger = 195000 + vData_y * 8192.0f /3.6f;
				gimbal_vision_yaw_tar_anger   = (vision_yaw_angle - yaw_angle) * 100;
				/*用于控制边小陀螺边移动*/
				if(Misc_Fabsf(vData.recv_data.speed_yaw) < 0.5f)
					vision_yaw_navigate = vData.recv_data.speed_yaw * 57.3f * vision_constant[0];
				else if(Misc_Fabsf(vData.recv_data.speed_yaw) < 1.0f)
					vision_yaw_navigate = vData.recv_data.speed_yaw * 57.3f * vision_constant[1];
				else
					vision_yaw_navigate = vData.recv_data.speed_yaw * 57.3f * vision_constant[2];	
		}
}

/**
  * @brief  发送视觉数据
  * @param  void
  * @retval void
  * @attention 1.发送当前的底盘速度与云台角度
               2.底盘正向速度解算：
                 已知底盘四个电机的转速rmp，求解底盘的速度。向前为x轴，向左为y轴，逆时针旋转为正 
  */
uint8_t ababa;
int32_t err_cnt = 0;
int32_t err_record = 0;
void Vision_SendData(void)
{
		uint8_t tmp_data[32];
//		Robot_ID = (uint8_t)JUDGE_u8GetRobotId();
    Sentry_HP = JUDGE_u16robot_HP(); 
    Remaining_Time = JUDGE_u16remaining_time();
	  Remaining_Bullet = JUDGE_u16bullet_remaining_num_17mm();
    Outpost_HP = JUDGE_u16outpost_HP();	
		Enemy_Outpost_HP =  JUDGE_u16enemyoutpost_HP();
		game_state = JUDGE_u8game_progress();
		Pitch_IMU_Angle = (Holder.Pitch_0x20B.Motor_Angle - 195000) / 8192.0f * 3.6f;
    Yaw_IMU_Angle = INS.Yaw;
		Chassis_IMU_Angle = mc_imu_yaw_chassis.Angle/10.0f - 180.0f;
		IntakeArea_ID = JUDGE_u8intakearea_id();
		PatrolArea_ID = JUDGE_u8patrolarea_id();
		if(JUDGE_u8game_progress() == 4) /* 比赛开始 */
		{
				if (Robot_ID <= 7 && Robot_ID > 0) shoot_state = 0x02;/*< 己方为红色*/
				else if (Robot_ID > 7) shoot_state = 0x01;	/*< 己方为蓝色*/
		}   
		else 
		{
				shoot_state = 0x00;
		}
		
		if(vData.recv_data.isSpining_Vision == 0x01 || spining_flag)
		{	
				spining_flag = 0x01;
				vData.recv_data.isSpining = 0x01;
		}/*视觉控制，比赛开始后第一次收到小陀螺标志位后一直保持小陀螺模式*/		
		
		if(Outpost_HP <= 0)
			spining_flag = 0x01;
		
		if(JUDGE_u8game_progress() != 4)
		{
			spining_flag = 0;
			vData.recv_data.isSpining = 0;
		} //视觉代码一坨大便，比赛不开始就不发数据，所以改在这，比赛未开始时控制小陀螺的标志位置零
		
		#ifndef DEBUG
			test_mode = 0;
		#endif
		
		switch(test_mode)
		{
			case 1:
				if(Remaining_Time < 390)Outpost_HP = 0;
				else Outpost_HP = 1500;
				if(Remaining_Time < 340)Enemy_Outpost_HP = 0;
				else Enemy_Outpost_HP = 1500;
			break;
			
			case 2:
				if(Remaining_Time < 390)Outpost_HP = 0;
				else Outpost_HP = 1500;
				if(Remaining_Time < 300)Enemy_Outpost_HP = 0;
				else Enemy_Outpost_HP = 1500;
			break;
			
			case 3:
				if(Remaining_Time < 400)Enemy_Outpost_HP = 0;
				else Enemy_Outpost_HP = 1500;
				if(Remaining_Time < 380)Outpost_HP = 0;
				else Outpost_HP = 1500;
			break;
			
			default:
			break;
		}
		
		#ifdef DEBUG
			Outpost_HP = 1500;
			Enemy_Outpost_HP = 1500;
			
		#endif
		
		if(IntakeArea_ID == 1)Exchanged_Bullet = Total_Exchange_Bullet;
		Total_Exchange_Bullet = ((420 - Remaining_Time) / 60) * 100;
		Available_Exchange_Bullet = Total_Exchange_Bullet - Exchanged_Bullet;
		
		if(Remaining_Time > 180)Available_Departurn_Time = 90;
		else Available_Departurn_Time = 30;
		if(Remaining_Time == 180)Departure_Time = Remaining_Time;
		if(Outpost_HP != 0 || PatrolArea_ID == 1)Departure_Time = Remaining_Time;
		Remaining_Spread_Time = Available_Departurn_Time - (Departure_Time - Remaining_Time);
		
		
		Robot_ID = 7;
		shoot_state = 0x02;//己方为红方
//		Robot_ID = 107;
//		shoot_state = 0x01;//己方为蓝方
		
    tmp_data[1] = shoot_state;  
		memcpy(&tmp_data[2], &Pitch_IMU_Angle, sizeof(Pitch_IMU_Angle));   /* 当前云台俯仰角 */
		memcpy(&tmp_data[6], &Yaw_IMU_Angle, sizeof(Yaw_IMU_Angle));      /* 当前云台偏航角 */
		memcpy(&tmp_data[10], &Chassis_IMU_Angle, sizeof(Chassis_IMU_Angle)); /*当前底盘偏航角*/
		memcpy(&tmp_data[14], &Sentry_HP, sizeof(Sentry_HP));              /* 哨兵血量 */
		memcpy(&tmp_data[16], &Remaining_Time, sizeof(Remaining_Time));    /* 剩余时间 */
		memcpy(&tmp_data[18], &Remaining_Spread_Time, sizeof(Remaining_Spread_Time)); /* 离开巡逻区时间 */
		memcpy(&tmp_data[20], &Outpost_HP, sizeof(Outpost_HP));							/* 前哨站血量 */
		memcpy(&tmp_data[22], &Available_Exchange_Bullet, sizeof(Available_Exchange_Bullet)); /*剩余可兑换发弹量*/
		tmp_data[24] = Holder_Command_ID;													/* 云台手发送指令 */
		tmp_data[25] = game_state;
		memcpy(&tmp_data[26], &Enemy_Outpost_HP, sizeof(Enemy_Outpost_HP));	
		memset(&tmp_data[28], 0, 3);                                       /* 无效数据 */
	
		/* 帧头帧尾 */
		tmp_data[0] = 0xFF;
		tmp_data[31] = 0x0D;
		
		ababa = CDC_Transmit_FS(tmp_data,sizeof(tmp_data));
		if(ababa != 0)err_cnt++;
}

void Vision_Judge(void)
{
		vision_protection++;
		if(vision_protection > 1000)
		{
				Vision_Protect();//一段时间未收到视觉数据清零
				vision_protection = 2000;
		}
			
}

void Vision_Protect(void)
{
		vData.recv_data.firesuggestion = 0;
		vData.recv_data.isNavigating = 0;
//		vData.recv_data.isSpining = 0;
//		vData.recv_data.isSpining_Vision = 0;
		vData.recv_data.Distance = 0;
		vData.recv_data.speed_x = 0;
		vData.recv_data.speed_y = 0;
		vData.recv_data.speed_yaw = 0;
}
uint16_t keyboard_cnt = 0;
uint8_t keyboard_flag = 0;
void Vision_Get_cmdkeyboard(void)
{
		keyboard_cnt ++;
		if(keyboard_flag != JUDGE_u8_robot_command() && JUDGE_u8game_progress() == 4)//当收到的指令与上次不同视为收到云台手指令
		{
			keyboard_cnt = 0;
			keyboard_flag = JUDGE_u8_robot_command();
		}
		Holder_Command_ID = JUDGE_u8_robot_command();
		if(keyboard_cnt >= 4000)//一段时间后将发送数据清零
		{
			keyboard_cnt = 4000;
			Holder_Command_ID = '0';
		}
		if(JUDGE_u8game_progress() != 4)
		{
				Holder_Command_ID = '0';
		}/*比赛未开始置零*/
}
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
