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
static uint8_t supercap_state_tx = CHASSIS_SUPERCAP;//CHASSIS_SUPERCAP;  /* ��������״̬ */
static float average_spin;         /* С����ƽ��ת�� */
static bool spin_ramp = true;      /* С����б�±�־λ */
/* function ------------------------------------------------------------------*/
/**
  * @brief  ��ȡ�����ǵĵ�������
  * @param  ����ָ��ṹ��
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
  * @brief  PID��ʼ��
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
  * @brief  PID����
  * @param  void
  * @retval void
  * @attention 
  */
void Chassis_PidCalculate(void)
{  
	 for(int i = 1;i <= 4;i++)
	 {
		  Chassis.M3508[i].Speed = (float)Chassis.M3508[i].Speed_Rx;
		  Chassis.M3508[i].Speed_Lpf = Chassis.M3508[i].Speed * 0.7f + Chassis.M3508[i].Speed_Lpf * 0.3f; //��ͨ�˲�
		 
		  Chassis.M3508[i].Current = (float)Chassis.M3508[i].Current_Rx;
	    Chassis.M3508[i].Current_Lpf = Chassis.M3508[i].Current * 0.7f + Chassis.M3508[i].Current_Lpf * 0.3f; //��ͨ�˲�
		 
      Chassis.M3508[i].TarCurrent = pid_increment_update(Chassis.M3508[i].TarSpeed, Chassis.M3508[i].Speed_Lpf, &Chassis.M3508[i].PidInc_Speed);
		  Chassis.M3508[i].Output = pid_increment_update(Chassis.M3508[i].TarCurrent, Chassis.M3508[i].Current_Lpf, &Chassis.M3508[i].PidInc_Current);
		 
	  /* ����޷� */
    Chassis.M3508[i].Output = constrain_int16_t(Chassis.M3508[i].Output, -Chassis_Out_Limit, Chassis_Out_Limit);
	 }
}

/**
  * @brief  ��ȡң�����ƶ�����
  * @param  ң������Ϣ�ṹ��ָ��
  * @retval void
  * @attention
			Ch2�������ң�Chassis.MoveData.Left��
			Ch3����ǰ��Chassis.MoveData.Front��
			Ch0������ת��Chassis.MoveData.ClockWise��
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
  * @brief  ������̨ģʽ
  * @param  ң������Ϣ�ṹ��
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
  * @brief  С����
  * @param  void
  * @retval void
  * @attention б������С����
  */
float DynamicSpin(void) 
{
	  static float spin_tmp;
	  if(vData.recv_data.isNavigating == 0x01)	spin_ramp = true;		
	
	  if(spin_ramp == true)
		{
		    spin_tmp = RAMP_float(average_spin, spin_tmp, 0.05f); /* б�º�����ʹת�ٻ������� */
			  if(spin_tmp >= average_spin) 
				{
				   spin_tmp = average_spin;
					 spin_ramp = false;
				}
		}
		return spin_tmp;					
}

/**
  * @brief  �ٶ�΢��
  * @param  void
  * @retval void
  * @attention ��̨�ְ����ٿ�
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
  * @brief ROS����ģʽ
  * @param  
  * @retval 
  * @attention 
  */
float RosControl(void)
{
	  float spin_tmp;
//    if (JUDGE_u8game_progress() == 4)    //��ʽ������ʼ
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
  * @brief  ����ģʽѡ��
  * @param  void
  * @retval void
  * @attention S1����Ϊң�������ƣ�S1����ΪС����/Ros״̬���м�Ϊֹͣ
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
								case SUP:   /* ������̨ */
//										spin_tmp = Follow_Holder(dir_angle); 
										spin_tmp = -RemoteMsg.Ch0/100;
								break;
															
								case SMID:  /* ����ֹͣ */
										Chassis_Protect();
								break;
								
								case SDOWN: /* ����ң���� */
										spin_tmp = RemoteMsg.Ch0/200;
								break;
						}
					break; 
							
					case SDOWN:
						bias_angle_shoot = -3070;
						switch(RemoteMsg.S2)
							{
								case SUP: /* ��������ģʽ������ģʽС����+���� */
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
								
								case SMID: /* ң��ģʽС���� */
										 move_state = HolderMode;	  
                     GetRemoteMoveData(RemoteMsg);								
										 spin_tmp = DynamicSpin();
								break;
								
								case SDOWN: /* ROS����*/
										 spin_tmp = RosControl();
								break;
							}           
				 break; 
							
				 case SMID: /* ����ֹͣ */
							bias_angle_shoot = -3070;
						  Chassis_Protect();
							if(RemoteMsg.S2 != 3) /* �ֶ��л���������ģʽ */
							{
							   supercap_state_tx = CHASSIS_POWER;
							}
							else
							{
							   supercap_state_tx = CHASSIS_SUPERCAP;
							}								
				 break; 			
			}
			
			/* ����޷� */	
			Chassis.MoveData.ClockWise = constrain_float(spin_tmp, -3*PI1, 3*PI1);	
}

/**
  * @brief  �������ݵ��̹��ʿ���
  * @param  void
  * @retval void
  * @attention  
  */
void Chassis_PowerControl(void)
{
			/* ���ͳ������ݿ������� */
			SuperCapCanTransmit();
	
      /* ��ѹ�㹻�ߡ���������ǰ������������ */
				if(Chassis.SuperCap.cap_vol >= 14000 && (supercap_state_tx == CHASSIS_SUPERCAP || Chassis.SuperCap.sup_state == 0))
				{
					 supercap_state_tx = CHASSIS_SUPERCAP;
						 
					/* ���ݵ��ݵ�ѹ����С����ת�� */
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
			else  /* ��ѹ������رյ��� */
			{							
				  Chassis.MoveData.Front = Chassis.MoveData.Front/2.0f;
				  Chassis.MoveData.Left = Chassis.MoveData.Left/2.0f;
					average_spin = 1.25f * PI1;		  //1.25
				
					/* ���ݻ��Զ���磬��ѹ�㹻��ʱ�л��ص���ģʽ */
					if(Chassis.SuperCap.cap_vol >= 20000 && supercap_state_tx == CHASSIS_POWER)
						 supercap_state_tx = CHASSIS_SUPERCAP;  
					else
						 supercap_state_tx = CHASSIS_POWER;
			}				
}

/**
  * @brief  �������ݷ���
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
  * @brief  �����˶�ѧ����
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
		
			/* �����������ٽ��� *//*�涨x��ǰ��y������ʱ��Ϊ��*/
			tmp_wheel_speed[0] =  (-front/sqrt(2) - left/sqrt(2) - clockwise * Chassis_r)/GAIN_F;  
			tmp_wheel_speed[1] = -(-front/sqrt(2) + left/sqrt(2) + clockwise * Chassis_r)/GAIN_F;
			tmp_wheel_speed[2] =  (-front/sqrt(2) + left/sqrt(2) - clockwise * Chassis_r)/GAIN_F;
			tmp_wheel_speed[3] = -(-front/sqrt(2) - left/sqrt(2) + clockwise * Chassis_r)/GAIN_F;
			/*ȫ���ֽ���*/
			/*1 0
			  3 2*/
			
			for(int j = 0; j <= 3; j++)
				 Chassis.M3508[j + 1].TarSpeed = tmp_wheel_speed[j];
//			WheelSpeedConstrait(tmp_wheel_speed); /* �ٶ��޷� */
}

/**
  * @brief  ������������
  * @param  void
  * @retval void
  * @attention 
  *    �������ĳ�����ٴ������ֵ��С����Сֵ�������
  *    �����������ŵ������ٶ����ڣ�����3������ҲҪ��ͬ�������š����ú����������ӽ��̣������������
  */
void WheelSpeedConstrait(float* tmp_wheel_speed)
{
		float limit_speed = fmax_wheel_speed;
		float tmp_max_speed = 1.0f;
		float tmp_min_speed = 1.0f;
		float cmp_index = 1.0f;

		for(int i = 0; i <= 3; i++)
		{   
				if (tmp_wheel_speed[i] > tmp_max_speed) tmp_max_speed = tmp_wheel_speed[i];  /* �����ֵ */
				if (tmp_wheel_speed[i] < tmp_min_speed) tmp_min_speed = tmp_wheel_speed[i];  /* ����Сֵ */
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
* @brief  CAN����
  * @param  void
  * @retval void
  * @attention ���ж�����
  */
void Chassis_CanTransmit(void)
{
    if (Observer.Tx.DR16_Rate > 15) /* ң����������������16ʱ�ſ������� */
    {     
       for(int j = 0,k = -1;j <= 6; j = j + 2)
	      {
					/* CAN ��ֵ */
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
  * @brief  ���̽���
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
//	  /* ����ģʽ�л�������0.2s */
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
  * @brief  ����ң��������
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
