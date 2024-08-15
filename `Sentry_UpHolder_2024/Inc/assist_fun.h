/**
  ******************************************************************************
  * @file    misc_func.h
  * @author  qj
  * @brief   部分辅助函数
  * @date     
  ******************************************************************************
  * @attention
  *
  * Copyright (c) CSU_RM_FYT
  * All rights reserved.
  *
  * This software component is licensed by SY under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MISC_FUNC_H
#define _MISC_FUNC_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include <stdint.h>
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
/* function ------------------------------------------------------------------*/

/* 限幅 */
//int constrain(int amt, int low, int high);
float constrain_float(float amt, float low, float high);
int16_t constrain_int16_t(int16_t amt, int16_t low, int16_t high);
int32_t Constrain_Int32_t(int32_t amt, int32_t low, int32_t high);
uint16_t Constrain_Uint16_t(uint16_t amt, uint16_t low, uint16_t high);

/* 斜坡 */
float RAMP_float( float final, float now, float ramp );
uint16_t RAMP_Uint16( uint16_t final, uint16_t now, uint16_t ramp );
int16_t Misc_RAMP_Int16( int16_t final, int16_t now, int16_t ramp );
int32_t Misc_RAMP_Int32( int32_t final, int32_t now, int32_t ramp );
float RampInc_float( float *buffer, float now, float ramp );

/* 数学 */
float fastCos(int angle);
float fastSin(int angle);
int16_t sign(float num);
float Quake_Sqrt(float x);
float Misc_Fabsf(float x);
int16_t Misc_Fabs16(int16_t x);
int32_t Misc_Fabs32(int32_t x);
float falE(float e,float alpha,float zeta);

/* 滤波 */
int16_t Misc_s16Recursive_Mean_Filter(int16_t get_data);
int16_t Misc_s16Recursive_Mean4_Filter(int16_t get_data, int16_t RM4_Filter_Buf[4]);
int32_t Misc_s32Recursive_Mean4_Filter(int32_t get_data, int32_t RM4_Filter_Buf[4]);
float Misc_fRecursive_Mean4_Filter(float get_data, float RM4_Filter_Buf[4]);

#ifdef __cplusplus
}
#endif

#endif
