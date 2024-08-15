#ifndef _SHOOT_CONFIG_H
#define _SHOOT_CONFIG_H
#include <stdint.h>
#include "config.h"
#include <stdbool.h>
/******************************************************************************
  * @brief  �������������ز���
  * @attention ÿ��ֻ�ܶ���һ�����β��������������ע�͵�debug��ض���
  *     1.�������2006��������ʽPID����
  *     �ٶȻ������þ���ʽPID������
  *     ���Է�����
  *         1.���� #define SHOOT_FREQ_DEBUG����Ƶ���ƣ���ͨ��debug�����޸Ĳ���
  *         2.���� #define SHOOT_FREQ_WAVE����Ƶ���ƣ�����������������
  *         3.���� #define SHOOT_SPEED_DEBUG, ���ٿ���
  *         4.���� #define SHOOT_SPEED_WAVE, ���ٲ���
  *         ����ʵ�ַ�����shoot.c�е�static void Shoot_PidRun(void)������
  *
  *     2.Ħ���ֵ��snail��������
  ****************************************************************************/
	
/* define --------------------------------------------------------------------*/	

/* variables -----------------------------------------------------------------*/
uint16_t Shoot_heat = 0;		/*< ǹ��ʵʱ����*/
int16_t shoot_rate = 10;	/*< ��Ƶ*/
int16_t shoot_freq = 45;  /* ң��״̬�Զ��������Զ�״̬����ݾ����ж� */
bool Shoot_cool = false;  /*< ǹ������������ȴ�ı�־λ*/      
bool Shoot_single = false;

/* ����PID���� */  
	float Shoot_Speed_kp = 4.0;//5
	float Shoot_Speed_ki = 0.04;
	float Shoot_Speed_kd = 12.0; //0.3
	float Shoot_Speed_kf = 0.0;
	float Shoot_Angle_kp = 800;
	float Shoot_Angle_ki = 0;
	float Shoot_Angle_kd = 2500;    //600    
	float Shoot_Angle_kf = 0.0;

/* Ħ����PID���� */
	float Shoot_3508_Speed_kp = 35.0;//5
	float Shoot_3508_Speed_ki = 0.0;
	float Shoot_3508_Speed_kd = 0.0; //0.3
	float Shoot_3508_Speed_kf = 0.0;
/*Ħ�������ٲ���*/
//�������
int16_t Shoot_Speed_Max_M3508_Lim15 = 4500;
int16_t Shoot_Speed_Max_M3508_Lim18 = 4950;
int16_t Shoot_Speed_Max_M3508_Lim22 = 5550;
int16_t Shoot_Speed_Max_M3508_Lim30 = 7000;

//�޷�
float shoot_kp_limit = 30000;
float shoot_ki_limit = 30000;
float shoot_pidlim = 30000;
float Shoot_Freq_out_Limit= 10000;
float shoot_2006_output_limit = 10000;

#endif

