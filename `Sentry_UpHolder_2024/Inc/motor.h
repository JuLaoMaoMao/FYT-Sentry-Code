/**
  ******************************************************************************
  * @file    
  * @author  zz
  * @brief   
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
#ifndef _MOTOR_H
#define _MOTOR_H


#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include <stdint.h>

typedef struct _M_CIRCLE_T
{
    int32_t Circle;
    int32_t Angle; 
}M_CIRCLE_T;


/* functions ！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！ */
float Misc_Fabsf(float x); //蒸斤峙痕方
void CircleContinue(M_CIRCLE_T *mc, uint16_t angle, uint16_t half_angle);

#ifdef __cplusplus
}
#endif

#endif /* */

