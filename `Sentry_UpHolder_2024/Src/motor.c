
/* Includes ------------------------------------------------------------------*/
#include "motor.h"
#define CIRCLE_FASTEST 1000

/* functions ！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！ */
/**
  * @brief  窮字郡澄銭偬算
  * @param  銭偬算潤更悶、輝念叔業峙
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

