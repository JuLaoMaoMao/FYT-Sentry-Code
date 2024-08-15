#ifndef _CHASSIS_CONFIG_H
#define _CHASSIS_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include <stdint.h>
#include "config.h"
#include <stdbool.h>
	 
/******************************************************************************
  * @brief  �������������ز���
  * @attention ÿ��ֻ�ܶ���һ�����β��������������ע�͵�debug��ض���
  ****************************************************************************/

/* ����״̬���� */
int8_t move_state = 0;	           /* �����˶�״̬ */
int16_t chassis_shift_cnt = 0;     /* ����״̬�л����� */
float clockwise = 0;               /* ������ת���ٶ� */
float front_temp = 0;              /* ������ǰ�ٶ� */
float left_temp = 0;	             /* ���������ٶ� */ 
float fmax_wheel_speed = 8000.0f;  /* M3508�޶����ת�٣�rpm */
/* ����������� */
float Chassis_r = 0.2765;              /* ���̰뾶r */
float Omin_r = 0.067;          /* ȫ���ְ뾶r */
float GAIN_F = 0.00037; 
/*
   �ٶȻ��㣺
   1rpm = 0.1047rad/s
   ȫ����ת��Ϊ1rpm�������ȫ��ʵ���ٶ�(�ع��ӷ���)rad/s���£�
   1rpm * 0.1047 * 0.067 ��= 0.007105���ٰѼ��ٱ����ȥ���ɵã�
   GAIN_F = 0.007105*(187/3591) = 0.00037
*/

/* ���̵��PID */ 
//�ٶȻ�
	float M3508_Speed_kp = 9.0f;  //8.0
  float M3508_Speed_ki = 0.1f; //0.01
  float M3508_Speed_kd = 12.0f; //12.0
//������
	float M3508_Current_kp = 0.8f;  //0.8f
  float M3508_Current_ki = 0.1f;  //0.1
  float M3508_Current_kd = 0.1f;  //0.1
	 
  float Chassis_Out_Limit = 30000.0f;

#ifdef __cplusplus
}
#endif

#endif
	
	

