
#include "pid.h"
#include "motor.h"
#include "assist_fun.h"
/**
  * @brief  ����ʽPID��ʼ��.
  * @param  PID�ṹ���ַ��P,I,D,�����޷��������޷�.
  * @note   .
  * @retval none.
  */
	
void pid_init_increment(PID_IncrementType *PID, float kp, float ki, float kd, float IncLim) //PID��ʼ��ϵ��
{
    PID->kp = kp;
    PID->ki = ki;
    PID->kd = kd;
    PID->IncLim = IncLim;
}

/**
  * @brief  ��ͨ������ʽPID�����Խ��������޷���.
  * @param  Ŀ��ֵ������ֵ��PID�ṹ���ַ.
  * @note   ���صĲ�������������ֱ������Ҫ�����ֵ.
  * @retval ��Ҫ�������.
  */
float pid_increment_update(float Target, float Current, PID_IncrementType *PID)
{
    float dErrP, dErrI, dErrD;

    PID->errNow = Target - Current; //�������Ŀ��-Ŀǰ���������β���ֵȡ��֣�
    
    dErrP = PID->errNow - PID->errOld1;		 //�����������----΢�֣������-��һ�����
    dErrI = PID->errNow;								   //������ַ��������������������
    dErrD = PID->errNow - 2 * PID->errOld1 + PID->errOld2; //����΢�ַ��������������������-2*һ�����΢��+�������΢��

    /*����ʽPID����*/
//	  if(Misc_Fabsf(PID->errNow) <= 300)
//		{
//			 PID->dCtrOut = 0;
//		}
//		else
//		{
       PID->dCtrOut = PID->kp * dErrP + PID->ki * dErrI + PID->kd * dErrD; //PID�ϳ������
//		}

    PID->errOld2 = PID->errOld1; //�������΢��
    PID->errOld1 = PID->errNow;  //һ�����΢��
    
    if (PID->dCtrOut < -PID->IncLim)
        PID->dCtrOut = -PID->IncLim;
    else if (PID->dCtrOut > PID->IncLim)
        PID->dCtrOut = PID->IncLim;

    PID->ctrOut += PID->dCtrOut;

    return PID->ctrOut;
}

/**
  * @brief  �ֶ�����ʽPID��ʼ��.
  * @param  PID�ṹ���ַ��P1,P2,P3,I,D,���ֶε�0��1���ɽ��ܵ���������ERRMAX��������ֱֵ�������ֵOUTMAX��.
  * @note   .
  * @retval none.
  */
void pid_init_increment_section(PID_IncrementType_section *PID, float *kp, float *ki, float kd, float *err,float IncLim) 
{
    PID->kp[0] = kp[0];
		PID->kp[1] = kp[1];
		PID->kp[2] = kp[2];
		PID->kp[3] = kp[3];
    PID->ki[0] = ki[0];
		PID->ki[1] = ki[1];
		PID->ki[2] = ki[2];
		PID->ki[3] = ki[3];
    PID->kd = kd;
		PID->err[0] = err[0];
		PID->err[1] = err[1];
		PID->err[2] = err[2];
    PID->IncLim = IncLim;
}

/**
  * @brief  �ֶ�����ʽPID�����Խ��������޷���.
  * @param  Ŀ��ֵ������ֵ��PID�ṹ���ַ.
  * @note   ���صĲ�������������ֱ������Ҫ�����ֵ.
  * @retval ��Ҫ�������.
  */
float pid_increment_update_section(float Target, float Current, PID_IncrementType_section *PID)
{
    float dErrP, dErrI, dErrD;

    PID->errNow = Target - Current; //�������Ŀ��-Ŀǰ���������β���ֵȡ��֣�

    dErrP = PID->errNow - PID->errOld1;					   //�����������----΢�֣������-��һ�����
    dErrI = PID->errNow;								   //������ַ��������������������
    dErrD = PID->errNow - 2 * PID->errOld1 + PID->errOld2; //����΢�ַ��������������������-2*һ�����΢��+�������΢��
 
//		if(Misc_Fabsf(PID->errNow) < 5)
//				dErrI = 0;
	
    /*����ʽPID����*/
	  if(Misc_Fabsf(PID->errNow)<PID->err[0])
				PID->dCtrOut += PID->kp[0] * dErrP + PID->ki[0] * dErrI + PID->kd * dErrD; //PID�ϳ������
		else if(Misc_Fabsf(PID->errNow)<PID->err[1]&&Misc_Fabsf(PID->errNow)>=PID->err[0])
				PID->dCtrOut += PID->kp[1] * dErrP + PID->ki[1] * dErrI + PID->kd * dErrD; //PID�ϳ������
		else if(Misc_Fabsf(PID->errNow)<PID->err[2]&&Misc_Fabsf(PID->errNow)>=PID->err[1])
				PID->dCtrOut += PID->kp[2] * dErrP + PID->ki[2] * dErrI + PID->kd * dErrD; //PID�ϳ������
		else
				PID->dCtrOut += PID->kp[3] * dErrP + PID->ki[3] * dErrI + PID->kd * dErrD;

    PID->errOld2 = PID->errOld1; //�������΢��
    PID->errOld1 = PID->errNow;  //һ�����΢��
		
		PID->dCtrOut = constrain_float(PID->dCtrOut,-PID->IncLim,PID->IncLim);

    return PID->dCtrOut;
}

/**
  * @brief  ����ʽPID��ʼ��.
  * @param  PID�ṹ���ַ��P,I,D,�����޷�,�����޷�
  * @note   .
  * @retval none.
  */
void pid_init_absolute(PID_AbsoluteType *PID, float kp, float ki, float kd, float kf, float errIlim, float errPLim, float out_min, float out_max)
{
    PID->kp = kp;
    PID->ki = ki;
    PID->kd = kd;
		PID->kf = kf;
    PID->errILim = errIlim;
    PID->errPLim = errPLim;
	  PID->out_min = out_min;
	  PID->out_max = out_max;
}

/**
  * @brief  ��ͨ����ʽPID.
  * @param  Ŀ��ֵ��ʵ��ֵ��PID�ṹ���ַ.
  * @note   .
  * @retval ��Ҫ�����ֵ.
  */
float pid_absolute_update(float Target, float Current, PID_AbsoluteType *PID)
{
    PID->errNow = Target - Current;
    PID->errP = PID->errNow; //��ȡ���ڵ�������kp����
    PID->errI += PID->errNow; //�����֣�����ki����
//		PID->errD = PID->errNow - PID->errOld; //���΢�֣�����kd����
		PID->errD = PID->Last_Current - Current;//΢������
		PID->errF = Target - PID->Last_Target;
	
    if (PID->errILim != 0) //�������޺�����
    {
				PID->errI = constrain_float(PID->errI,-PID->errILim,PID->errILim);
    }
    /* ����������޺����� */
    if (PID->errPLim != 0)
    {
				PID->errP = constrain_float(PID->errP,-PID->errPLim,PID->errPLim);
    }
		PID->errF = constrain_float(PID->errF,-500,500);
		
    PID->errOld = PID->errNow; //�������ڵ����
    PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD + PID->kf * PID->errF; //�������ʽPID���
    PID->ctrOut = constrain_float(PID->ctrOut,PID->out_min,PID->out_max);
		
		PID->Last_Target = Target;
		PID->errOld = PID->errNow; //�������ڵ����
		PID->Last_Current = Current;
		return PID->ctrOut;
}

/**
  * @brief  �����ݺ�����ͨ����ʽpid
  * @param  Ŀ��ֵ��ʵ��ֵ��pid�ṹ���ַ
  * @retval ��Ҫ�����ֵ
  * @attention
  */
float pid_absolute_update_Fal(float Target, float Current, PID_AbsoluteType *PID,float alpha,float zeta,float Gain_error)
{
    PID->errNow = Target - Current;
    PID->errP = Gain_error*falE(PID->errNow,alpha,zeta);  /*< ��ȡ���ڵ�������kp���� */
    PID->errI += PID->errNow; /*< �����֣�����ki���� */
    
    if (PID->errILim != 0)     /*< �������޺����� */
    {
        if (PID->errI > PID->errILim)
        {
            PID->errI = PID->errILim;
        }
        else if (PID->errI < -PID->errILim)
        {
            PID->errI = -PID->errILim;
        } 
    }
    
    /* ����������޺����� */
    if (PID->errPLim != 0)
    {
        if (PID->errP > PID->errPLim)
        {
            PID->errP = PID->errPLim;
        }
        else if (PID->errP < -PID->errPLim)
        {
            PID->errP = -PID->errPLim;
        }
    }
    
    PID->errD = PID->errNow - PID->errOld; /*< ���΢�֣�����kd���� */
    PID->errOld = PID->errNow; /*< �������ڵ���� */
    PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD; /*< �������ʽpid��� */
    
    /* ������޺����� */
    if (PID->ctrOut != 0)
    {
        if (PID->ctrOut > PID->out_max)
        {
            PID->ctrOut = PID->out_max;
        }
        else if (PID->ctrOut < PID->out_min)
        {
            PID->ctrOut = PID->out_min;
        }
    }
    
    return PID->ctrOut;
}

/**
  * @brief  ����Kp����ʽPID��ʼ��.
  * @param  PID�ṹ���ַ��P1,P2,P3,I,D,�����޷������ֶε�0��1���ɽ��ܵ���������ERRMAX��������ֱֵ�������ֵOUTMAX��.
  * @note   .
  * @retval none.
  */
void pid_init_absolute_section_kp(PID_AbsoluteType_section_kp* PID, float *kp, float ki, float kd, float *err,float errIlimit, float OUTMAX)
{
    PID->tempkp[0] = kp[0];
    PID->tempkp[1] = kp[1];
    PID->tempkp[2] = kp[2];
    PID->err[0] = err[0];
    PID->err[1] = err[1];
	  PID->ki = ki;
    PID->kd	= kd;
    PID->errIlimit = errIlimit;
    PID->errNow= 0;
    PID->errP= 0;
    PID->errI= 0;
    PID->errD= 0;
    PID->errOld= 0;
    PID->ctrOut= 0;
    PID->OUTMAX = OUTMAX;
}

/**
  * @brief  ����Kp����ʽPID
  * @param  Ŀ��ֵ������ֵ��PID�ṹ���ַ
  * @note   ����һ����Χ��ֱ�������ֵ����δʹ�û��ַ���.
  * @retval ���ؼ���������ֵ.
  */
float pid_absolute_section_update_kp(float Target,float Current,PID_AbsoluteType_section_kp* PID)
{
    PID->errNow = Target - Current;

    if(Misc_Fabsf(PID->errNow) < PID->err[0])
		{
        PID->kp = PID->tempkp[0];
		}
    else if(Misc_Fabsf(PID->errNow) >= PID->err[0] && Misc_Fabsf(PID->errNow) < PID->err[1])
		{
        PID->kp = PID->tempkp[1];
		}
    else
		{
        PID->kp = PID->tempkp[2];
		}

    PID->errP = PID->errNow;  //��ȡ���ڵ�������kp����

    PID->errI += PID->errNow; //�����֣�����ki����

    if(PID->errIlimit != 0)	   //΢�����޺�����
    {
        if(PID->errI >= PID->errIlimit)
            PID->errI =  PID->errIlimit;
        else if(PID->errI <= -PID->errIlimit)
            PID->errI = -PID->errIlimit;
    }

    PID->errD = PID->errNow - PID->errOld;//���΢�֣�����kd����

    PID->errOld = PID->errNow;	//�������ڵ����

    PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//�������ʽPID���

		if (PID->ctrOut >= PID->OUTMAX)
				PID->ctrOut =  PID->OUTMAX;
		if (PID->ctrOut <= -PID->OUTMAX)
				PID->ctrOut = -PID->OUTMAX; 

		return PID->ctrOut;
}

/**
  * @brief  ���ξ���ʽPID��ʼ��.
  * @param  PID�ṹ���ַ��P1,P2,P3,I,D,�����޷������ֶε�0��1���ɽ��ܵ���������ERRMAX��������ֱֵ�������ֵOUTMAX��.
  * @note   .
  * @retval none.
  */
void pid_init_absolute_section(PID_AbsoluteType_section* PID, float *kp, float *ki, float kd, float *err,float errIlimit, float OUTMAX)
{
    PID->tempkp[0] = kp[0];
    PID->tempkp[1] = kp[1];
    PID->tempkp[2] = kp[2];
    PID->tempki[0] = ki[0];
    PID->tempki[1] = ki[1];
    PID->tempki[2] = ki[2];
    PID->err[0] = err[0];
    PID->err[1] = err[1];
    PID->kd	= kd;
    PID->errIlimit = errIlimit;
    PID->errNow= 0;
    PID->errP= 0;
    PID->errI= 0;
    PID->errD= 0;
    PID->errOld= 0;
    PID->ctrOut= 0;
    PID->OUTMAX = OUTMAX;
}

/**
  * @brief  ���ξ���ʽPID
  * @param  Ŀ��ֵ������ֵ��PID�ṹ���ַ
  * @note   ����һ����Χ��ֱ�������ֵ����δʹ�û��ַ���.
  * @retval ���ؼ���������ֵ.
  */
float pid_absolute_section_update(float Target,float Current,PID_AbsoluteType_section* PID)
{
    PID->errNow = Target - Current;

    if(Misc_Fabsf(PID->errNow) < PID->err[0])
		{
        PID->kp = PID->tempkp[0];
			  PID->ki = PID->tempki[0];
		}
    else if(Misc_Fabsf(PID->errNow) >= PID->err[0] && Misc_Fabsf(PID->errNow) <= PID->err[1])
		{
        PID->kp = PID->tempkp[1];
			  PID->ki = PID->tempki[1];
		}
    else
		{
        PID->kp = PID->tempkp[2];
			  PID->ki = PID->tempki[2];
		}

    PID->errP = PID->errNow;  //��ȡ���ڵ�������kp����

    PID->errI += PID->errNow; //�����֣�����ki����

    if(PID->errIlimit != 0)	   //΢�����޺�����
    {
        if(PID->errI >= PID->errIlimit)
            PID->errI =  PID->errIlimit;
        else if(PID->errI <= -PID->errIlimit)
            PID->errI = -PID->errIlimit;
    }

    PID->errD = PID->errNow - PID->errOld;//���΢�֣�����kd����

    PID->errOld = PID->errNow;	//�������ڵ����

    PID->ctrOut = PID->kp * PID->errP + PID->ki * PID->errI + PID->kd * PID->errD;//�������ʽPID���

		if (PID->ctrOut >= PID->OUTMAX)
				PID->ctrOut =  PID->OUTMAX;
		if (PID->ctrOut <= -PID->OUTMAX)
				PID->ctrOut = -PID->OUTMAX; 

		return PID->ctrOut;
}

float Holder_Yaw_T = 0.6,Holder_Pitch_T = 3;
float Holder_Yaw_last_in = 0,Holder_Pitch_last_in = 0;
float Holder_Yaw_forward(float in)
{
	float out;
	out=(in-Holder_Yaw_last_in)*Holder_Yaw_T;
	Holder_Yaw_last_in=in;
	return out;
}
float Holder_Pitch_forward(float in)
{
	float out;
	out=(in-Holder_Pitch_last_in)*Holder_Pitch_T;
	Holder_Pitch_last_in=in;
	return out;
}
