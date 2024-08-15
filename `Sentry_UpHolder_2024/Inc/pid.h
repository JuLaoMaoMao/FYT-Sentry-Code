#ifndef __PID_H
#define __PID_H
#include <stdio.h>
#include <stdint.h>
#define max(a, b)			(a>b? a:b)
#define min(a, b)			(a<b? a:b)
#define range(x, a, b)		(min(max(x, a), b))//a-��Сֵ   b-���ֵ

typedef struct
{
    /*PID�㷨�ӿڱ��������ڸ��û���ȡ���޸�PID�㷨������*/
    float kp; //����ϵ��
    float ki; //����ϵ��
    float kd; //΢��ϵ��

    float errNow;  //��ǰ�����
    float dCtrOut; //�����������
    float ctrOut;  //�������

    float IncLim; //�����޷�
    /*PID�㷨�ڲ���������ֵ�����޸�*/
    float errOld1;
    float errOld2;
} PID_IncrementType;    /* ����ʽpid */

typedef struct
{
/*PID�㷨�ӿڱ��������ڸ��û���ȡ���޸�PID�㷨������*/
    float kp[4]; //����ϵ��
    float ki[4]; //����ϵ��
    float kd; //΢��ϵ��
	
		float err[3];//����������ֶ�

    float errNow;  //��ǰ�����
    float dCtrOut; //�����������
    float ctrOut;  //�������

    float IncLim; //�����޷�
    /*PID�㷨�ڲ���������ֵ�����޸�*/
    float errOld1;
    float errOld2;
		float lastdeltadev;//����ȫ΢��ʹ�ã���һ��΢�����
		float lastoutput;
} PID_IncrementType_section;    /* �ֶ�ʽ����ʽpid */

typedef struct
{
    float kp;
    float ki;
    float kd;
		float kf;
    float errILim;   //��������
    float errPLim; //��������
	  float out_min;   //�������
    float out_max;   //�������
    float errNow;
    float errOld;
    float errP;
    float errI;
    float errD;
		float errF;
    float ctrOut;
		float Last_Target;
		float Last_Current;
} PID_AbsoluteType;         /* ����ʽpid */

typedef struct
{
    float tempkp[3];
    float err[2];
    float kp;
    float ki;
    float kd;
    float errIlimit;
    float errNow;
    float errOld;
    float errP;
    float errI;
		float Item;
    float errD;
    float ctrOut;
    float ERRMAX;
    float OUTMAX;
    float AccErr;

} PID_AbsoluteType_section_kp;

typedef struct
{
    float tempkp[3];
    float tempki[3];	
    float err[2];
    float kp;
    float ki;
    float kd;
    float errIlimit;
    float errNow;
    float errOld;
    float errP;
    float errI;
		float Item;
    float errD;
    float ctrOut;
    float ERRMAX;
    float OUTMAX;
    float AccErr;

} PID_AbsoluteType_section;

/* PID������ */
void pid_init_increment(PID_IncrementType *PID, float kp, float ki, float kd, float IncLim);
void pid_init_increment_section(PID_IncrementType_section *PID, float *kp, float *ki, float kd, float *err,float IncLim);

void pid_init_absolute(PID_AbsoluteType *PID, float kp, float ki, float kd, float kf,float errIlim, float errPLim, float out_min, float out_max);
void pid_init_absolute_section_kp(PID_AbsoluteType_section_kp* PID, float *kp, float ki, float kd, float *err,float errIlimit,float OUTMAX);
void pid_init_absolute_section(PID_AbsoluteType_section* PID, float *kp, float *ki, float kd, float *err,float errIlimit, float OUTMAX);

float pid_increment_update(float Target, float Current, PID_IncrementType *PID);
float pid_increment_update_section(float Target, float Current, PID_IncrementType_section *PID);

float pid_absolute_section_update_kp(float Target,float Current,PID_AbsoluteType_section_kp* PID);
float pid_absolute_section_update(float Target,float Current,PID_AbsoluteType_section* PID);
float pid_absolute_update(float Target, float Current, PID_AbsoluteType *PID);
float pid_absolute_update_Fal(float Target, float Current, PID_AbsoluteType *PID,float alpha,float zeta,float gain_error);

float constrain_float(float amt, float low, float high);
float Holder_Yaw_forward(float in);
float Holder_Pitch_forward(float in);
#endif

