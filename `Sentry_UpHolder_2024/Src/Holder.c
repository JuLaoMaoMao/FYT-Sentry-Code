/**
  ******************************************************************************
  * @file    Holder.c
  * @author  zz
  * @brief   
  * @date    2022-10-30
  ******************************************************************************
  * @attention 
**/
/* includes ------------------------------------------------------------------*/
#include "assist_fun.h"
#include "can.h"
#include "Holder.h"
#include "holder_config.h"
#include "message.h"
#include "motor.h"
#include "Judge.h"
#include "pid.h"
#include "tim.h"
#include "usart.h"
#include "Vision.h"
//#include "MahonyAHRS.h"
#include "BMI088driver.h"
#include "ins_task.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
Holder_t Holder = {0};
float vision_pitch, vision_yaw, vision_z;            
float ff1,ff2;
float Holder_Cruise_Angle = -180000;
uint16_t Holder_Armor[4] = {3400,1350,7475,5500};
/* function ------------------------------------------------------------------*/
/**
  * @brief  ��ȡ�����ǵ���̨����
  * @param  ����ָ��ṹ��
  * @retval void
  * @attention 
  */
void IMU_GetHolderData(void)
{ 
	  /* ��ȡ������ƫ���� */
    Holder.Yaw_0x20A.IMU_Angle = yaw_angle * 100.0f;
    Holder.Yaw_0x20A.IMU_Speed = INS.Gyro[2] * gain_f;
	  Holder.Yaw_0x20A.IMU_Angle_Lpf = Holder.Yaw_0x20A.IMU_Angle * 0.85 + Holder.Yaw_0x20A.IMU_Angle_Lpf * 0.15;
	  Holder.Yaw_0x20A.IMU_Speed_Lpf = Holder.Yaw_0x20A.IMU_Speed * 0.85 + Holder.Yaw_0x20A.IMU_Speed_Lpf * 0.15;

	  /* ��ȡ�����Ǹ����� */
    Holder.Pitch_0x20B.IMU_Angle = pitch_angle * 100.0f ;  
    Holder.Pitch_0x20B.IMU_Speed = INS.Gyro[1] * gain_f;
	  Holder.Pitch_0x20B.IMU_Angle_Lpf = Holder.Pitch_0x20B.IMU_Angle * 0.85 + Holder.Pitch_0x20B.IMU_Angle_Lpf * 0.15;
	  Holder.Pitch_0x20B.IMU_Speed_Lpf = Holder.Pitch_0x20B.IMU_Speed * 0.85 + Holder.Pitch_0x20B.IMU_Speed_Lpf * 0.15;	  
}

/**
  * @brief  ��̨PID��ʼ��
  * @param  void
  * @retval void
  * @attention 
  */
void Holder_PidInit(void)
{
				/* ��̨���� */
				/*������*/
				pid_init_absolute(&Holder.Pitch_0x20B.PidAbs_Position, Pitch_Angle_kp, Pitch_Angle_ki, Pitch_Angle_kd,Pitch_Angle_kf, Pitch_Angle_ILimit, Pitch_Angle_PLimit, -holder_pidlim, holder_pidlim);
				pid_init_absolute(&Holder.Pitch_0x20B.PidAbs_Speed, Pitch_Speed_kp, Pitch_Speed_ki, Pitch_Speed_kd,Pitch_Speed_kf, Pitch_Speed_ILimit, Pitch_Speed_PLimit, -holder_pidlim, holder_pidlim);
	
				/*������*/
				pid_init_absolute(&Holder.Pitch_0x20B.PidAbs_Position_Shoot, Pitch_Angle_kp_Shoot, Pitch_Angle_ki_Shoot, Pitch_Angle_kd_Shoot, Pitch_Angle_kf_Shoot, Pitch_Angle_ILimit, Pitch_Angle_PLimit, -holder_pidlim, holder_pidlim);
				pid_init_absolute(&Holder.Pitch_0x20B.PidAbs_Speed_Shoot, Pitch_Speed_kp_Shoot, Pitch_Speed_ki_Shoot, Pitch_Speed_kd_Shoot, Pitch_Speed_kf_Shoot, Pitch_Angle_ILimit, Pitch_Angle_PLimit, -holder_pidlim, holder_pidlim);
	
				/* ��̨ƫ�� */
	      /*������*/
	      pid_init_absolute(&Holder.Yaw_0x20A.PidAbs_Position, Yaw_Angle_kp, Yaw_Angle_ki, Yaw_Angle_kd,Yaw_Angle_kf, Yaw_Angle_ILimit, Yaw_Angle_PLimit, -holder_pidlim, holder_pidlim);
				pid_init_absolute(&Holder.Yaw_0x20A.PidAbs_Speed, Yaw_Speed_kp, Yaw_Speed_ki , Yaw_Speed_kd,Yaw_Speed_kf, Yaw_Speed_ILimit, Yaw_Speed_PLimit, -holder_pidlim, holder_pidlim);		 
				
				/*������*/
				pid_init_absolute(&Holder.Yaw_0x20A.PidAbs_Position_Shoot, Yaw_Angle_kp_Shoot, Yaw_Angle_ki_Shoot, Yaw_Angle_kd_Shoot, Yaw_Angle_kf_Shoot, Yaw_Angle_ILimit, Yaw_Angle_Shoot_PLimit, -holder_pidlim, holder_pidlim);
				pid_init_absolute(&Holder.Yaw_0x20A.PidAbs_Speed_Shoot, Yaw_Speed_kp_Shoot, Yaw_Speed_ki_Shoot, Yaw_Speed_kd_Shoot, Yaw_Speed_kf_Shoot, Yaw_Angle_ILimit, Yaw_Angle_Shoot_PLimit, -holder_pidlim, holder_pidlim);		 
}

/**
  * @brief  PID����
  * @param  void
  * @retval voi70000d
  * @attention 
  */
void Holder_PID_Calculate(REMOTE_DATA_T RemoteMsg)
{ 
			if(vData_z == 0 || vData_z == -1 || vData.recv_data.isNavigating == 0x01)
			{
					/* ƫ����� */    
					Holder.Yaw_0x20A.TarSpeed = pid_absolute_update(Holder.Yaw_0x20A.TarAngle, Holder.Yaw_0x20A.IMU_Angle_Lpf, &Holder.Yaw_0x20A.PidAbs_Position);
					Holder.Yaw_0x20A.Output = pid_absolute_update(Holder.Yaw_0x20A.TarSpeed, Holder.Yaw_0x20A.IMU_Speed_Lpf, &Holder.Yaw_0x20A.PidAbs_Speed);
					Holder.Yaw_0x20A.Output = Constrain_Int32_t(Holder.Yaw_0x20A.Output, -holder_pidlim, holder_pidlim);
				
					/* ������� */
					Holder.Pitch_0x20B.TarSpeed = pid_absolute_update(Holder.Pitch_0x20B.TarAngle, Holder.Pitch_0x20B.Motor_Angle, &Holder.Pitch_0x20B.PidAbs_Position);//2040Ϊˮƽ����Ƕ�
					Holder.Pitch_0x20B.Output = pid_absolute_update(Holder.Pitch_0x20B.TarSpeed, Holder.Pitch_0x20B.IMU_Speed_Lpf, &Holder.Pitch_0x20B.PidAbs_Speed);
					Holder.Pitch_0x20B.Output = constrain_int16_t(Holder.Pitch_0x20B.Output, -holder_pidlim, holder_pidlim);
			}
			else
			{
					/*	ƫ����� */    
					Holder.Yaw_0x20A.TarSpeed = pid_absolute_update(Holder.Yaw_0x20A.TarAngle, Holder.Yaw_0x20A.IMU_Angle_Lpf, &Holder.Yaw_0x20A.PidAbs_Position_Shoot);
					Holder.Yaw_0x20A.Output = pid_absolute_update(Holder.Yaw_0x20A.TarSpeed, Holder.Yaw_0x20A.IMU_Speed_Lpf, &Holder.Yaw_0x20A.PidAbs_Speed_Shoot);
					Holder.Yaw_0x20A.Output = Constrain_Int32_t(Holder.Yaw_0x20A.Output, -holder_pidlim, holder_pidlim);
				
					/* ������� */
					Holder.Pitch_0x20B.TarSpeed = pid_absolute_update(Holder.Pitch_0x20B.TarAngle, Holder.Pitch_0x20B.Motor_Angle, &Holder.Pitch_0x20B.PidAbs_Position_Shoot);//-1400Ϊˮƽ����Ƕ�(�Ƕȴ���)
					Holder.Pitch_0x20B.Output = pid_absolute_update(Holder.Pitch_0x20B.TarSpeed, Holder.Pitch_0x20B.IMU_Speed_Lpf, &Holder.Pitch_0x20B.PidAbs_Speed_Shoot);
					Holder.Pitch_0x20B.Output = constrain_int16_t(Holder.Pitch_0x20B.Output, -holder_pidlim, holder_pidlim);
					
			}
	 
}

/**
  * @brief  �Ӿ����ݴ���
  * @param  void
  * @retval void
  * @attention
  */
M_CIRCLE_T v_imu_yaw 	 = {0};
float gimbal_vision_yaw_tar_anger,gimbal_vision_pitch_tar_anger;
void Holder_Vision_deal(void)
{
		static int16_t vData_y_cnt;   /*< �쳣����*/
		static int16_t vData_x_cnt;
		/* ���� */

				if ((vData_z == -1) && (vision_z > 0))	/*< ǰһ֡���һ֡���ж��Ƿ��֡*/
				{
						vData_y_cnt++;
						if (vData_y_cnt > 10)
						{
								vision_pitch = 195000;
								vData_y_cnt = 0;
						}
				}
				else 
				{
						vData_y_cnt = 0;
						vision_z = vData_z;
					  vision_pitch = gimbal_vision_pitch_tar_anger;					
				}

				if ((vData_z == -1) && (vision_z > 0))
				{
						vData_x_cnt++;
						if (vData_x_cnt > 10)
						{
								vision_yaw = 0;									
								vData_x_cnt = 0;
						}
				}
				else 
				{
						vData_x_cnt = 0;
						vision_z = vData_z;
						vision_yaw = gimbal_vision_yaw_tar_anger ;					
				}	
}

/**
  * @brief  �Ӿ�����ģʽ
  * @param  void
  * @retval void
  * @attention
  */
void Holder_Vision(REMOTE_DATA_T RDMsg)  
{
	 static int16_t vision_protect;  /* ��֡����*/
	 if(JUDGE_u8game_progress() == 4 || Remote_Data.S2 == 1)      //��ʽ������ʼ
	 { 
			if(vData.recv_data.isNavigating == 0x01 && Remote_Data.S2 != 1)
			{
//				 Holder_Forward();
				 Holder_Navigate();
			}
			else
			{
					Holder_Vision_deal();

					if(vData_z > 0.1f && vData_z <= 10.0f)
						 vision_protect = 300; /* ������֡300ms��Ӧ�����Ӿ���֡ʱ�� */
					else
					{
							vision_protect--;
							if(vision_protect <= 0) vision_protect = 0;
					} 
					if (vision_protect > 0)
					{
							Holder_follow();
					}
					else
					{
//							Holder_Protect();
							Holder_Cruise();
					}
			}				   
	 } 
	 else
		 Holder_Protect();
}

void Holder_Vision_Manual(REMOTE_DATA_T RDMsg)
{
	 static int16_t vision_protect;  /* ��֡����*/
	 if(JUDGE_u8game_progress() == 4 || Remote_Data.S2 == 1)      //��ʽ������ʼ
	 { 
			if(vData.recv_data.isNavigating == 0x01 && Remote_Data.S2 != 1)
			{
//				 Holder_Forward();
				 Holder_Navigate();
			}
			else
			{
					Holder_Vision_deal();

					if(vData_z > 0.1f && vData_z <= 10.0f)
						 vision_protect = 300; /* ������֡300ms��Ӧ�����Ӿ���֡ʱ�� */
					else
					{
							vision_protect--;
							if(vision_protect <= 0) vision_protect = 0;
					} 
					if (vision_protect > 0)
					{
							Holder_follow();
					}
					else
					{
//							Holder_Protect();
							Holder_Manual(RDMsg);
					}
			}				   
	 } 
	 else
		 Holder_Protect();
}
/**
  * @brief  ��̨ң��
  * @param  void
  * @retval void
  * @attention
  */
void Holder_Manual(REMOTE_DATA_T RemoteMsg)
{   
    Holder.Yaw_0x20A.TarAngle -= 0.4 * Remote_Data.Ch0;	  
    Holder.Pitch_0x20B.TarAngle += 0.4 * Remote_Data.Ch1;	
	  if(Holder.Pitch_0x20B.TarAngle >= Pitch_Max) Holder.Pitch_0x20B.TarAngle = Pitch_Max;
		if(Holder.Pitch_0x20B.TarAngle <= Pitch_Min) Holder.Pitch_0x20B.TarAngle = Pitch_Min; 
}

/**
  * @brief  ��̨����
  * @param  void
  * @retval void
  * @attention ��̨ƫ��������̣������Ƕȹ̶�
  */
void Holder_Forward(void)
{
		 int32_t dir_angle_navi = 1000 * GetDirAngle_Shoot(); /* �����������ƫ���Ƕ� */
	   Holder.Yaw_0x20A.TarAngle = Holder.Yaw_0x20A.IMU_Angle - dir_angle_navi;
	   Holder.Pitch_0x20B.TarAngle = Pitch_Fix_Angle;
}

/**
  * @brief  С���ݵ���ʱ��̨���Ӿ�����
  * @param  void
  * @retval void
  * @attention ��������̨����Ϊ���򣬸����Ƕȹ̶�
  */
float vision_yaw_navigate;
void Holder_Navigate(void)
{
		Holder.Yaw_0x20A.TarAngle = Holder.Yaw_0x20A.IMU_Angle + vision_yaw_navigate;
	  Holder.Pitch_0x20B.TarAngle = Pitch_Fix_Angle;
}


/**
  * @brief  �Զ�Ѳ��
  * @param  void
  * @retval void
  * @attention 
  */
void Holder_Cruise(void)
{	
	    float Cruise_Min_Tmp, Cruise_Max_Tmp, pitch_delta;
			Cruise_Min_Tmp = Pitch_Cruise_Min[1];
			Cruise_Max_Tmp = Pitch_Cruise_Max[1];
			pitch_delta = Pitch_Delta[1];
			if(Holder.Pitch_0x20B.Motor_Angle >= Cruise_Max_Tmp)	Holder.Pitch_0x20B.Direction = 0;
			if(Holder.Pitch_0x20B.Motor_Angle <= Cruise_Min_Tmp)	Holder.Pitch_0x20B.Direction = 1;
			if(Misc_Fabsf(Holder.Pitch_0x20B.TarAngle - Holder.Pitch_0x20B.Motor_Angle)/1000 <= 2)  //����ᶶ���Ӹ��޷�
			{
				if(Holder.Pitch_0x20B.Direction)	
					 Holder.Pitch_0x20B.TarAngle += pitch_delta;//30
				else 	
					 Holder.Pitch_0x20B.TarAngle -= pitch_delta;
			}	
				 
			   Holder.Yaw_0x20A.TarAngle -= 120;
}

/**
  * @brief  ��̨����
  * @param  void
  * @retval void
  * @attention �����Ӿ�	
  */
void Holder_follow(void)
{	
		   /* ƫ����������̨���׷�ɢ����һ�±��� */ 	
			if(Misc_Fabsf(Holder.Yaw_0x20A.TarAngle - Holder.Yaw_0x20A.IMU_Angle)/1000.0f <= 80.0f) 
			{ 							
						if(Misc_Fabsf(vision_yaw) >= 4000.0f) //4.0
							Holder.Yaw_0x20A.TarAngle = Holder.Yaw_0x20A.IMU_Angle + vision_yaw * 1.15f; //����

						else if(Misc_Fabsf(vision_yaw) >= 1500.0f)
							Holder.Yaw_0x20A.TarAngle = Holder.Yaw_0x20A.IMU_Angle + vision_yaw * 1.08f; //
						
						else
							Holder.Yaw_0x20A.TarAngle = Holder.Yaw_0x20A.IMU_Angle + vision_yaw * 1.5f; //
			}
				/* ���� */ 	
			 Holder.Pitch_0x20B.TarAngle = vision_pitch;
			 if(Holder.Pitch_0x20B.TarAngle >= Pitch_Max) Holder.Pitch_0x20B.TarAngle = Pitch_Max;		
			 if(Holder.Pitch_0x20B.TarAngle <= Pitch_Min) Holder.Pitch_0x20B.TarAngle = Pitch_Min;	
		
}

/**
  * @brief  ��ȡ����װ�װ�ƫ���
  * @param  ID �ܻ���װ�װ�ID
  * @retval float
  * @attention 
  */
float GetDirAngle_Holder(uint8_t ID)
{
		float dir_angle;
		dir_angle = Holder.Yaw_0x20A.Position.Angle - Holder_Armor[ID];
    dir_angle -= dir_angle>4096? 8192:0;
    dir_angle += dir_angle<-4096? 8192:0;
	  dir_angle = dir_angle * 360/8192.0f; 
	  return dir_angle;
}

/**
  * @brief  ��̨ģʽѡ��
  * @param  ������Ϣ�ṹ��
  * @retval void
  * @attention void
  */
void Holder_ChooseMode(REMOTE_DATA_T RemoteMsg)
{
//		hh1++;
    switch(Remote_Data.S1)
    {
        case SUP:
					 bias_angle_shoot = -3070;//-1000
//					 hh1++;
           if(Remote_Data.S2 == SUP)
						 Holder_Manual(RemoteMsg); /* 1(up)������̨ң�أ�2(down)������̨ң�� */
					 
					 else if(Remote_Data.S2 == SMID)
						 Holder_Manual(RemoteMsg);
						 
					 else if(Remote_Data.S2 == SDOWN) 
						 Holder_Forward(); /* ��̨���򻯣���̨������̣���ƫ����Ը�����̨Ϊ׼*/
//						 Holder_Cruise();
//						Holder_Protect();
				break;
        
				case SMID:
//						hh1++;
						bias_angle_shoot = -3070;//-3070
						if(Remote_Data.S2 == SUP)
							Holder_Vision_Manual(RemoteMsg);
						
						else if(Remote_Data.S2 == SMID)
							Holder_Protect();
						
						else if(Remote_Data.S2 == SDOWN)
							 Holder_Cruise();
				break;
						
        case SDOWN:
						bias_angle_shoot = -3070;
            if(Remote_Data.S2 == SUP) 
						   Holder_Vision(RemoteMsg);
						
						else if(Remote_Data.S2 == SMID) 
							 Holder_Vision(RemoteMsg);
						
						else if(Remote_Data.S2 == SDOWN)
							 Holder_Vision(RemoteMsg); /* �Ӿ����� */
				break;
						
				default:
//					hh1++;
				break;
		}
}

/**
* @brief  CAN���ͺ���
  * @param  void
  * @retval void
  * @attention �ж��е���
  */
void Holder_CanTransmit(void)
{   
    if(flag) /*< ң����������������16ʱ�ſ������� *///Observer.Tx.DR16_Rate > 12  flag
     {           
			  Holder.CanData[0] = (uint8_t)(Holder.Yaw_0x20A.Output >> 8);
        Holder.CanData[1] = (uint8_t)(Holder.Yaw_0x20A.Output);
        Holder.CanData[2] = (uint8_t)(Holder.Pitch_0x20B.Output >> 8);
			  Holder.CanData[3] = (uint8_t)(Holder.Pitch_0x20B.Output);
//				Holder.CanData[2] = 0;
//        Holder.CanData[3] = 0;
//        Holder.CanData[4] = 0;
//			  Holder.CanData[5] = 0;
      }
		 
    else    
			Holder_Protect(); /*< �ر�ң��������̨Ŀ��Ƕ�һֱ���ֵ�ǰ״̬ */

    CAN1_Transmit(0x1FF,Holder.CanData);
//		CAN2_Transmit(0x1FF,Holder.CanData);
}

/**
  * @brief  ��̨���̿���
  * @param  void
  * @retval void
  * @attention 
  */
void Holder_Process(REMOTE_DATA_T RemoteMsg)
{
		
	  IMU_GetHolderData();
    Holder_ChooseMode(RemoteMsg);
	  Holder_PID_Calculate(RemoteMsg);
}

/**
* @brief  ��̨����
  * @param  void
  * @retval void
  * @attention �ж��е���
  */
void Holder_Protect(void) 
{
				Holder.Pitch_0x20B.TarAngle = Holder.Pitch_0x20B.Motor_Angle;
	      Holder.Pitch_0x20B.TarSpeed = 0;
				Holder.Pitch_0x20B.PidAbs_Position.ctrOut = 0;
				Holder.Pitch_0x20B.PidAbs_Position.errNow = 0;
				Holder.Pitch_0x20B.PidAbs_Position.errOld = 0;
	      Holder.Pitch_0x20B.PidAbs_Position.errI = 0;
	      Holder.Pitch_0x20B.PidAbs_Position_Section.ctrOut = 0;
	      Holder.Pitch_0x20B.PidAbs_Position_Section.errNow = 0;
	      Holder.Pitch_0x20B.PidAbs_Position_Section.errOld = 0;
	      Holder.Pitch_0x20B.PidAbs_Position_Section.errI = 0;
	      Holder.Pitch_0x20B.PidAbs_Position_Section.errP = 0;
				Holder.Pitch_0x20B.PidAbs_Position_Section.errD = 0;
				Holder.Pitch_0x20B.PidAbs_Speed.ctrOut = 0;
				Holder.Pitch_0x20B.PidAbs_Speed.errNow = 0;
				Holder.Pitch_0x20B.PidAbs_Speed.errOld = 0;
				Holder.Pitch_0x20B.PidAbs_Speed.errI = 0;
	
				Holder.Yaw_0x20A.TarAngle = Holder.Yaw_0x20A.IMU_Angle;
			  Holder.Yaw_0x20A.TarSpeed = 0;
				Holder.Yaw_0x20A.PidAbs_Position.ctrOut = 0;
				Holder.Yaw_0x20A.PidAbs_Position.errNow = 0;
				Holder.Yaw_0x20A.PidAbs_Position.errOld = 0;
				Holder.Yaw_0x20A.PidAbs_Position.errI = 0;
	      Holder.Yaw_0x20A.PidAbs_Position_Section.ctrOut = 0;
	      Holder.Yaw_0x20A.PidAbs_Position_Section.errNow = 0;
	      Holder.Yaw_0x20A.PidAbs_Position_Section.errOld = 0;
	      Holder.Yaw_0x20A.PidAbs_Position_Section.errI = 0;
	      Holder.Yaw_0x20A.PidAbs_Position_Section.errP = 0;
				Holder.Yaw_0x20A.PidAbs_Position_Section.errD = 0;
				Holder.Yaw_0x20A.PidAbs_Speed.ctrOut = 0;
				Holder.Yaw_0x20A.PidAbs_Speed.errNow = 0;
				Holder.Yaw_0x20A.PidAbs_Speed.errOld = 0;
				Holder.Yaw_0x20A.PidAbs_Speed.errI = 0;
				 
			  memset(Holder.CanData,0,sizeof(Holder.CanData)); /* ���CAN */
}

