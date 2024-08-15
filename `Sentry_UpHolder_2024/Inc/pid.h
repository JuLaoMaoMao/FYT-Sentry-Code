#ifndef __PID_H
#define __PID_H
#include <stdio.h>
#include <stdint.h>
#define max(a, b)			(a>b? a:b)
#define min(a, b)			(a<b? a:b)
#define range(x, a, b)		(min(max(x, a), b))//a-最小值   b-最大值

typedef struct
{
    /*PID算法接口变量，用于给用户获取或修改PID算法的特性*/
    float kp; //比例系数
    float ki; //积分系数
    float kd; //微分系数

    float errNow;  //当前的误差
    float dCtrOut; //控制增量输出
    float ctrOut;  //控制输出

    float IncLim; //增量限幅
    /*PID算法内部变量，其值不能修改*/
    float errOld1;
    float errOld2;
} PID_IncrementType;    /* 增量式pid */

typedef struct
{
/*PID算法接口变量，用于给用户获取或修改PID算法的特性*/
    float kp[4]; //比例系数
    float ki[4]; //积分系数
    float kd; //微分系数
	
		float err[3];//根据这个误差分段

    float errNow;  //当前的误差
    float dCtrOut; //控制增量输出
    float ctrOut;  //控制输出

    float IncLim; //增量限幅
    /*PID算法内部变量，其值不能修改*/
    float errOld1;
    float errOld2;
		float lastdeltadev;//不完全微分使用，上一拍微分输出
		float lastoutput;
} PID_IncrementType_section;    /* 分段式增量式pid */

typedef struct
{
    float kp;
    float ki;
    float kd;
		float kf;
    float errILim;   //积分上限
    float errPLim; //比例上限
	  float out_min;   //输出上限
    float out_max;   //输出下限
    float errNow;
    float errOld;
    float errP;
    float errI;
    float errD;
		float errF;
    float ctrOut;
		float Last_Target;
		float Last_Current;
} PID_AbsoluteType;         /* 绝对式pid */

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

/* PID控制器 */
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

