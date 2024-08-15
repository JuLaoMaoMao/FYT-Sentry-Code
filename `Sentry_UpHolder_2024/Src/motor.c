
/* Includes ------------------------------------------------------------------*/
#include "motor.h"
#define CIRCLE_FASTEST 1000

/* functions �������������������������������������������������������������������������������������������������������������������� */
/**
  * @brief  �������������
  * @param  �������ṹ�塢��ǰ�Ƕ�ֵ
  * @retval void
  * @attention
  */
void CircleContinue(M_CIRCLE_T *mc, uint16_t angle, uint16_t half_angle)
{
    if (mc->Angle - angle < -half_angle)
    {
        mc->Circle--;
    }
		else if (mc->Angle - angle > half_angle)
    {
        mc->Circle++;
    }
    mc->Angle = angle;
}

