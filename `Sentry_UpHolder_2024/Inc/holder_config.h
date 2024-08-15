#ifndef _HOLDER_CONFIG_H
#define _HOLDER_CONFIG_H

#include <stdint.h>
#include "config.h"

/******************************************************************************
  * @brief  ��̨������ز���
  * @attention ˫�ջ����� 
  *     �⻷���ǶȻ������ñ�ṹPI����������С�������ӿ���Ӧ
  *     �ڻ����ٶȻ������þ���ʽPID������
  *     ���Է�����
  *         1.���� #define HOLDER_DEBUG����ͨ��debug�����޸Ĳ���
  *         2.���� #define HOLDER_PITCH_WAVE�������PITCH�Შ��
  *         3.���� #define HOLDER_YAW_WAVE, �����YAW�Შ��
  *         4.���� #define HOLDER_VISION_WAVE, ������Ӿ�����
  *         ����ʵ�ַ�����holder.c�е�static void Holder_Pid_Manual(void)������
  *
  *         ÿ��ֻ�ܶ���һ�����β��������������ע�͵�debug��ض���
  ****************************************************************************/
/* Sentry data */

///*��������*/
///* ���� */
//	float Pitch_Angle_kp = 0.22f; //0.18f
//	float Pitch_Angle_ki = 0.001f; //0.0015f
//	float Pitch_Angle_kd = 0.0f; //0.09f
//	float Pitch_Angle_kf = 2.0f;
//	float Pitch_Speed_kp = 23.0f; //15.0f
//	float Pitch_Speed_ki = 0.05f; //0.001
//	float Pitch_Speed_kd = 0.0f; //0.15f
//	float Pitch_Speed_kf = 0.0f;
//	
///* ƫ�� */		
//	float Yaw_Angle_kp = 0.15f; //0.09//0.095
//	float Yaw_Angle_ki = 0.00f; //0
//	float Yaw_Angle_kd = 0.1f; //0.1   
//	float Yaw_Angle_kf = 0.0f;
//	float Yaw_Speed_kp = 75.0f; //20//32.5
//  float Yaw_Speed_ki = 0.00f; //3.0
//  float Yaw_Speed_kd = 0.0f; //1.0
//	float Yaw_Speed_kf = 1.0f;
	
/* ���� */
	float Pitch_Speed_kp = 22.0f; //10.0f
	float Pitch_Speed_ki = 0.05f; //0.0f
	float Pitch_Speed_kd = 0.5f; //0.0f
	float Pitch_Speed_kf = 4.0f; //0.0f
	float Pitch_Angle_kp = 0.18f; //0.10f
	float Pitch_Angle_ki = 0.005f; //0.035f
	float Pitch_Angle_kd = 0.0f; //0.4f
	float Pitch_Angle_kf = 2.0f; //0.0f
/*�������*/
	float Pitch_Speed_kp_Shoot = 20.0f; //23.0f
	float Pitch_Speed_ki_Shoot = 0.1f; //0.05f
	float Pitch_Speed_kd_Shoot = 0.5f; //0.0f
	float Pitch_Speed_kf_Shoot = 4.0f; //0.0f
	float Pitch_Angle_kp_Shoot = 0.16f; //0.22f
	float Pitch_Angle_ki_Shoot = 0.005f; //0.001f
	float Pitch_Angle_kd_Shoot = 0.0f; //0.09f
	float Pitch_Angle_kf_Shoot = 2.0f; //2.0f

	
/* ƫ�� */		
	float Yaw_Speed_kp = 30.0f; //50
  float Yaw_Speed_ki = 0.05f; //0.0
  float Yaw_Speed_kd = 1.0f; //0.0
	float Yaw_Speed_kf = 1.0f; //2.0
	float Yaw_Angle_kp = 0.15f; //0.16
	float Yaw_Angle_ki = 0.0f; //0.0
	float Yaw_Angle_kd = 0.5f; //0.0   
	float Yaw_Angle_kf = 0.5f; //2.0

/*�������*/
	float Yaw_Speed_kp_Shoot = 55.0f; //65
  float Yaw_Speed_ki_Shoot = 0.0f; //0.12
  float Yaw_Speed_kd_Shoot = 20.0f; //0.9
	float Yaw_Speed_kf_Shoot = 4.0f; //4.0
	float Yaw_Angle_kp_Shoot = 0.22f; //0.23
	float Yaw_Angle_ki_Shoot = 0.0f; //0.0
	float Yaw_Angle_kd_Shoot = 12.0f; //2.5   
	float Yaw_Angle_kf_Shoot = 4.5f; //4.5 


//�޷�
	float Pitch_Angle_PLimit = 99999.0f;  /*< �ǶȻ�P��������޷� */
	float Pitch_Angle_ILimit = 99999.0f;  /*< �ǶȻ�I��������޷� */
	float Pitch_Speed_PLimit = 99999.0f;  /*< �ٶȻ�P��������޷� */
	float Pitch_Speed_ILimit = 99999.0f;  /*< �ٶȻ�I��������޷� */

	float Yaw_Angle_Shoot_PLimit = 99999.0f;
	float Yaw_Angle_PLimit = 49999.0f;  //34999.0 /*< �ǶȻ�P��������޷� */
	float Yaw_Angle_ILimit = 99999.0f;  /*< �ǶȻ�I��������޷� */
	float Yaw_Speed_Shoot_PLimit = 99999.0f;
	float Yaw_Speed_PLimit = 49999.0f;  //49999.0 /*< �ٶȻ�P��������޷� */
	float Yaw_Speed_ILimit = 99999.0f;  /*< �ٶȻ�I��������޷� */
	
	float holder_pidlim = 30000; /* pid����޷���30000 */

//Ѳ��ɨ�跶Χ
int32_t Pitch_Cruise_Min[2] = {170000, 170000};//172000, 182500  
int32_t Pitch_Cruise_Max[2] = {228000, 228000};//193500, 185500
int32_t Pitch_Delta[2] = {50, 120};
int32_t Yaw_Cruise_Max = 50000;
int32_t Yaw_Cruise_Min = -50000;
//ң�������Ӿ���Χ
int32_t Pitch_Min = 170000;//197200
int32_t Pitch_Max = 228000;//160000
int32_t Pitch_Fix_Angle = 195000;
int32_t Pitch_Outpost = 195000;
//1348 1837 
int32_t Yaw_Delta_Max = 500;//100;
int32_t Yaw_Delta_Min = 500;//800;

float gain_f = 573.0;
#endif /*< ifdef HOLDER_CONFIG_H*/


