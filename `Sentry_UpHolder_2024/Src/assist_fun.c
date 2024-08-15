/**
  ******************************************************************************
  * @file    
  * @author  qj
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
#include <math.h>
#include "assist_fun.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables ---------------------------------------------------------------- */
static float fast_cos_buf_[91] = { 1,
    0.999848,0.999391,0.99863,0.997564,0.996195,0.994522,0.992546,0.990268,
    0.987688,0.984808,0.981627,0.978148,0.97437,0.970296,0.965926,0.961262,
    0.956305,0.951057,0.945519,0.939693,0.93358,0.927184,0.920505,0.913545,
    0.906308,0.898794,0.891007,0.882948,0.87462,0.866025,0.857167,0.848048,
    0.838671,0.829038,0.819152,0.809017,0.798635,0.788011,0.777146,0.766044,
    0.75471,0.743145,0.731354,0.71934,0.707107,0.694658,0.681998,0.669131,
    0.656059,0.642788,0.62932,0.615661,0.601815,0.587785,0.573576,0.559193,
    0.544639,0.529919,0.515038,0.5,0.48481,0.469471,0.45399,0.438371,
    0.422618,0.406737,0.390731,0.374606,0.358368,0.34202,0.325568,0.309017,
    0.292372,0.275637,0.258819,0.241922,0.224951,0.207912,0.190809,0.173648,
    0.156434,0.139173,0.121869,0.104528,0.0871556,0.0697563,0.0523358,
    0.0348993,0.0174522,-1.73205e-07};

/***************************************�޷�***********************************/

/**
  * @brief  �޷�
  * @param  Ŀ���������Сֵ�����ֵ
  * @retval int�����޷�ֵ
  * @attention �޷������Ƚϳ��ã�����������ѭ��ִ��
  */
//int constrain(int amt, int low, int high)
//{
//    return amt<low? low:(amt>high? high:amt); /*< �ʺű��ʽ���б�if else�� */
//}

float constrain_float(float amt, float low, float high)
{
	return amt<low? low:(amt>high? high:amt); /*< �ʺű��ʽ���б�if else�� */
}

int16_t constrain_int16_t(int16_t amt, int16_t low, int16_t high)
{
	return amt<low? low:(amt>high? high:amt); /*< �ʺű��ʽ���б�if else�� */
}

int32_t Constrain_Int32_t(int32_t amt, int32_t low, int32_t high)
{
	return amt<low? low: (amt>high? high:amt); /*< �ʺű��ʽ���б�if else�� */
}

uint16_t Constrain_Uint16_t(uint16_t amt, uint16_t low, uint16_t high)
{
	return amt<low? low:(amt>high? high:amt); /*< �ʺű��ʽ���б�if else�� */
}

/***************************************б�±仯***********************************/

/**
  * @brief  б�º���,ʹĿ�����ֵ������������ֵ
  * @param  �����������,��ǰ���,�仯�ٶ�(ÿ�����ӵ�ֵ)
  * @retval ��ǰ���
  * @attention  
  */
float RAMP_float( float final, float now, float ramp )
{
    float buffer = 0;

    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {  
            now += ramp;
        }   
        else
        {
            now += buffer;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }

    return now;
}


int32_t Misc_RAMP_Int32( int32_t final, int32_t now, int32_t ramp )
{
    int32_t buffer = 0;

    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {  
            now += ramp;
        }   
        else
        {
            now += buffer;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }

    return now;
}


int16_t Misc_RAMP_Int16( int16_t final, int16_t now, int16_t ramp )
{
    int16_t buffer = 0;

    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {  
            now += ramp;
        }   
        else
        {
            now += buffer;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }

    return now;
}

/**
  * @brief  б�º���,ʹĿ�����ֵ������������ֵ
  * @param  �����������,��ǰ���,�仯�ٶ�(Խ��Խ��)
  * @retval ��ǰ���
  * @attention  
  */
uint16_t RAMP_Uint16( uint16_t final, uint16_t now, uint16_t ramp )
{
    int16_t buffer = 0;

    buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)
        {  
            now += ramp;
        }   
        else
        {
            now += buffer;
        }
    }
    else
    {
        if (buffer < -ramp)
        {
            now += -ramp;
        }
        else
        {
            now += buffer;
        }
    }

    return now;
}

/**
  * @brief  б�º���,ʹĿ�����ֵ��������ָ������ֵ
  * @param  Ҫ�ڵ�ǰ��������ۼӵ�ֵ,Ŀ�������,��������
  * @retval Ŀ�������
  * @attention  
  *             
*/
float RampInc_float( float *buffer, float now, float ramp )
{

    if (*buffer > 0)
    {
        if (*buffer > ramp)
        {  
            now     += ramp;
            *buffer -= ramp;
        }   
        else
        {
            now     += *buffer;
            *buffer  = 0;
        }
    }
    else
    {
        if (*buffer < -ramp)
        {
            now     += -ramp;
            *buffer -= -ramp;
        }
        else
        {
            now     += *buffer;
            *buffer  = 0;
        }
    }
    
    return now;
}


/***************************************��ѧ***********************************/

/**
  * @brief  ���ټ���cos
  * @param  �����Ƕ�ֵ����λ���ȡ�
  * @retval ����ֵ
  * @attention �������Ǻ��������ԺͶԳ��ԣ�ֻ��ȡ90����ֵ֪���ܼ�������нǶ�ֵ
  */
float fastCos(int angle)
{
	if (angle>=0 && angle <= 90)
	{
		return fast_cos_buf_[angle];
	}
	else if (angle > 90 && angle <=180)
	{
		return -(fast_cos_buf_[180-angle]);
	}
	else if (angle > 180 && angle <=360)
	{
		return fastCos(360-angle);
	}
	else if (angle > 360)
	{
		return fastCos(angle - 360);
	}
	else if (angle < 0)
	{
		return (fastCos(-angle));
	}
	return 0;
}

/**
  * @brief  ���ټ���sin
  * @param  �����Ƕ�ֵ����λ���ȡ�
  * @retval ����ֵ
  * @attention �������Ǻ��������ԺͶԳ��ԣ�ֻ��ȡ90����ֵ֪���ܼ�������нǶ�ֵ
  */
float fastSin(int angle)
{
	return fastCos(90.0f - angle);
}

/**
  * @brief  ���ź���
  * @param  ����ֵ
  * @retval ���ź������ֵ
  * @attention   
  */
int16_t sign(float num)
{
    if (num > 1E-6f)
    {
        return 1;
    }
    else if (num < -1E-6f)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief  Quake-III Arena (����֮��3) ���ش���
  * @param  x
  * @retval ����x
  * @attention  ����д�Ŀ���ƽ����    
  */
float Quake_Sqrt(float x)
{
    if(x == 0) return 0; 
    float result = x; 
    float xhalf = 0.5f*result; 
    int i = *(int*)&result; 

    i = 0x5f3759df - (i>>1); 
    result = *(float*)&i; 
    result = result*(1.5f-xhalf*result*result);
    result = result*(1.5f-xhalf*result*result); 
    return 1.0f/result; 
}

/**
  * @brief  floatȡ����ֵ
  * @param  x
  * @retval |x|
  * @attention  
  */
float Misc_Fabsf(float x)
{
    float y;
    y = x>=0? x:-x;
    return y;
}

/**
  * @brief  Intȡ����ֵ
  * @param  x
  * @retval |x|
  * @attention  
  */
int16_t Misc_Fabs16(int16_t x)
{
    int16_t y;
    y = x>0? x:-x;
    return y;
}

int32_t Misc_Fabs32(int32_t x)
{
    int32_t y;
    y = x>0? x:-x;
    return y;
}

/**
  * @brief  ԭ�㸽���������Զε������ݴκ���
  * @param  �����������ʡ��������䳤��
  * @retval �����Ժ��ֵ
  * @attention   
  */
float falE(float e,float alpha,float zeta)
{
    int16_t s=0;
    float output=0;
    s=(sign(e+zeta)-sign(e-zeta))/2;
    output=e*s/(powf(zeta,1-alpha))+powf(Misc_Fabsf(e),alpha)*sign(e)*(1-s);
    return output;
}

/***************************************�źŴ���***********************************/
/**
  * @brief  ���ƣ�������ƽ���˲�
  * @param  
  * @retval void
  * @attention 
  */
#define  RM_BUF_LEN                         4 /*��������ĳ���*/
int16_t RM_Filter_Buf[RM_BUF_LEN];            /*�������ݵĵ�������*/

int16_t Misc_s16Recursive_Mean_Filter(int16_t get_data)
{
    int16_t sum = 0;
    uint8_t i;
    
    for(i = 0; i < RM_BUF_LEN - 1; i++) 
    {
		RM_Filter_Buf[i] = RM_Filter_Buf[i+1];  /*������������������һλ ��һ�������ӵ�*/
		sum += RM_Filter_Buf[i];                /*������������д����һ��ǰ���������ݵĺ�*/
	}
    
    RM_Filter_Buf[RM_BUF_LEN - 1] = get_data;
    
    sum += RM_Filter_Buf[RM_BUF_LEN - 1];                     /*������������������ݵĺ�*/
    sum = sum / RM_BUF_LEN;
    return sum;
}

int16_t Misc_s16Recursive_Mean4_Filter(int16_t get_data, int16_t RM4_Filter_Buf[4])
{
    int16_t sum = 0;
    uint8_t i;
    
    for(i = 0; i < 3; i++) 
    {
		RM4_Filter_Buf[i] = RM4_Filter_Buf[i+1];  /*������������������һλ ��һ�������ӵ�*/
		sum += RM4_Filter_Buf[i];                 /*������������г����һ��ǰ���������ݵĺ�*/
	 }
    
    RM4_Filter_Buf[3] = get_data;
    
    sum =  RM4_Filter_Buf[0]*0.1 + RM4_Filter_Buf[1]*0.2 +RM4_Filter_Buf[2]*0.3 + RM4_Filter_Buf[3]*0.4;
    return sum;
}

int32_t Misc_s32Recursive_Mean4_Filter(int32_t get_data, int32_t RM4_Filter_Buf[4])
{
    int32_t sum = 0;
    uint8_t i;
    
    for(i = 0; i < 3; i++) 
    {
		RM4_Filter_Buf[i] = RM4_Filter_Buf[i+1];  /*������������������һλ ��һ�������ӵ�*/
		sum += RM4_Filter_Buf[i];                 /*������������г����һ��ǰ���������ݵĺ�*/
	 }
    
    RM4_Filter_Buf[3] = get_data;
    
    sum =  RM4_Filter_Buf[0]*0.1 + RM4_Filter_Buf[1]*0.2 +RM4_Filter_Buf[2]*0.3 + RM4_Filter_Buf[3]*0.4;
    return sum;
}

float Misc_fRecursive_Mean4_Filter(float get_data, float RM4_Filter_Buf[4])
{
    float sum = 0;
    uint8_t i;
    
    for(i = 0; i < 3; i++) 
    {
		RM4_Filter_Buf[i] = RM4_Filter_Buf[i+1];  /*������������������һλ ��һ�������ӵ�*/
		sum += RM4_Filter_Buf[i];                 /*������������д����һ��ǰ���������ݵĺ�*/
	}
    
    RM4_Filter_Buf[3] = get_data;
    
    sum =  RM4_Filter_Buf[0]*0.1f + RM4_Filter_Buf[1]*0.2f +RM4_Filter_Buf[2]*0.3f + RM4_Filter_Buf[3]*0.4f;
    return sum;
}
