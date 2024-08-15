/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MESSAGE_H
#define _MESSAGE_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "motor.h"
	 
/* typedef -------------------------------------------------------------------*/

	 
typedef struct 
{
	uint16_t w;
	uint16_t s;
	uint16_t a;
	uint16_t d;	
  uint16_t shift;
	uint16_t ctrl;
	uint16_t q;
	uint16_t e;
	uint16_t r;
	uint16_t f;
	uint16_t g;
	uint16_t z;
	uint16_t x;
	uint16_t c;
	uint16_t v;
	uint16_t b;
}Kb16_t,*Kb16_p;

typedef	struct _REMOTE_DATA_T
{
	int16_t	    Ch0;	
	int16_t	    Ch1;	
	int16_t	    Ch2;	
	int16_t	    Ch3;
	uint8_t	    S1;
	uint8_t	    S2;	
		
	int16_t	    Mouse_x;			
	int16_t	    Mouse_y;			
	uint8_t	    MouseClick_left;		
	uint8_t	    MouseClick_right;
	
    uint16_t    Key;
    Kb16_t      KeyBoard;
    
    int16_t     Wheel;
}REMOTE_DATA_T;

typedef struct _Holder_Data_t
{
    int16_t gyro[3];
    float angle[3];
	  float Net_Angle;
    float speed[3];
}Holder_Data_T;

typedef struct _Chassis_Data_t
{
		int16_t gyro[3];
		float angle[3];
}Chassis_Data_T;


/* define --------------------------------------------------------------------*/
#define KEY_PRESSED_OFFSET_W	    (1U << 0)
#define KEY_PRESSED_OFFSET_S	    (1U << 1)
#define KEY_PRESSED_OFFSET_A	    (1U << 2)
#define KEY_PRESSED_OFFSET_D	    (1U << 3)
#define KEY_PRESSED_OFFSET_SHIFT	(1U << 4)
#define KEY_PRESSED_OFFSET_CTRL	    (1U << 5)
#define KEY_PRESSED_OFFSET_Q	    (1U << 6)
#define KEY_PRESSED_OFFSET_E	    (1U << 7)
#define KEY_PRESSED_OFFSET_R	    (1U << 8)
#define KEY_PRESSED_OFFSET_F	    (1U << 9)
#define KEY_PRESSED_OFFSET_G	    (1U << 10)
#define KEY_PRESSED_OFFSET_Z	    (1U << 11)
#define KEY_PRESSED_OFFSET_X	    (1U << 12)
#define KEY_PRESSED_OFFSET_C	    (1U << 13)
#define KEY_PRESSED_OFFSET_V	    (1U << 14)
#define KEY_PRESSED_OFFSET_B	    (1U << 15)
/* variables -----------------------------------------------------------------*/
//extern M_CIRCLE_T mc_imu_yaw;
//extern M_CIRCLE_T mc_imu_pitch;
extern M_CIRCLE_T mc_imu_yaw_chassis;
extern REMOTE_DATA_T Remote_Data;
/* function ------------------------------------------------------------------*/
void Msg_RemoteDataProcess(REMOTE_DATA_T *RemoteMsg);
void Msg_HolderDataProcess(Holder_Data_T *HolderMsg);
void Msg_ChassisDataProcess(Chassis_Data_T *ChassisMsg);
void getIMUEular(float *eular_tmp);
void BoardCommunication(REMOTE_DATA_T *RemoteMsg);
void DownControl(void);
void DownControl2(void);
void DownControl3(void);
#ifdef __cplusplus
}
#endif

#endif /* */


