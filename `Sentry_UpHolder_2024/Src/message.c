/* includes ------------------------------------------------------------------*/
#include "message.h"
#include "motor.h"
#include "dma.h"
#include "usart.h"
#include "IMU_HI.h"
#include "can.h"
#include "Vision.h"
#include "tim.h"
#include "Judge_Rx.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/


/* variables -----------------------------------------------------------------*/
/*�����ǲ���*/
//M_CIRCLE_T mc_imu_yaw = {0};
//M_CIRCLE_T mc_imu_pitch = {0};
M_CIRCLE_T mc_imu_yaw_chassis = {0};
float eular[3] = {0};
int16_t gyro[3] = {0};
int16_t test_speed = 200;
int16_t dir_angle;
uint8_t game_flag,robot_ID;
int16_t vision_x,vision_y,vision_yaw1;
int16_t navigate,spining,firesuggestion;
REMOTE_DATA_T Remote_Data = {0};
/* function ------------------------------------------------------------------*/
void Msg_RemoteDataProcess(REMOTE_DATA_T *RDMsg)
{    
//    RDMsg->Ch0  = (  (int16_t) uart1_buf[0]       | ( (int16_t) uart1_buf[1]  << 8 )) & 0x07FF;
//    RDMsg->Ch0 -= 1024;
		RDMsg->Ch0 = Remote_Data.Ch0;
//    RDMsg->Ch1  = (( (int16_t) uart1_buf[1] >> 3) | ( (int16_t) uart1_buf[2]  << 5 )) & 0x07FF;
//    RDMsg->Ch1 -= 1024;
		RDMsg->Ch1 = Remote_Data.Ch1;
    RDMsg->Ch2  = (( (int16_t) uart1_buf[2] >> 6) | ( (int16_t) uart1_buf[3]  << 2 )  
                                                  | ( (int16_t) uart1_buf[4]  << 10)) & 0x07FF;
    RDMsg->Ch2 -= 1024;
    RDMsg->Ch3  = (( (int16_t) uart1_buf[4] >> 1) | ( (int16_t) uart1_buf[5]  << 7 )) & 0x07FF;
    RDMsg->Ch3 -= 1024;
//    RDMsg->S1   = (            uart1_buf[5] >> 6)                                     & 0x03;
		RDMsg->S1 = Remote_Data.S1;
//    RDMsg->S2   = (            uart1_buf[5] >> 4)                                     & 0x03;
		RDMsg->S2 = Remote_Data.S2;
         
    RDMsg->Mouse_x = ( (int16_t) uart1_buf[6] | (int16_t) uart1_buf[7] << 8);
    RDMsg->Mouse_y = ( (int16_t) uart1_buf[8] | (int16_t) uart1_buf[9] << 8);
       
    RDMsg->MouseClick_left  = uart1_buf[12];
    RDMsg->MouseClick_right = uart1_buf[13]; 
      
    RDMsg->Key     = ( (int16_t) uart1_buf[14] |   (int16_t) uart1_buf[15] << 8 );
    RDMsg->Wheel   = ( (int16_t) uart1_buf[16] | ( (int16_t) uart1_buf[17] << 8 )) & 0x07FF;
    RDMsg->Wheel   = -RDMsg->Wheel + 1024;

    if(RDMsg->Ch0 > 660 || RDMsg->Ch0 < -660)
        RDMsg->Ch0 = 0;
    if(RDMsg->Ch1 > 660 || RDMsg->Ch1 < -660)
        RDMsg->Ch1 = 0;
    if(RDMsg->Ch2 > 660 || RDMsg->Ch2 < -660)
        RDMsg->Ch2 = 0;
    if(RDMsg->Ch3 > 660 || RDMsg->Ch3 < -660)
        RDMsg->Ch3 = 0;
    if(RDMsg->Wheel > 660 || RDMsg->Wheel < -660)
        RDMsg->Wheel = 0;    
    
//    RDMsg->KeyBoard.w     = RDMsg->Key & KEY_PRESSED_OFFSET_W;
//    RDMsg->KeyBoard.s     = (RDMsg->Key & KEY_PRESSED_OFFSET_S)>>1;
//    RDMsg->KeyBoard.a     = (RDMsg->Key & KEY_PRESSED_OFFSET_A)>>2;
//    RDMsg->KeyBoard.d     = (RDMsg->Key & KEY_PRESSED_OFFSET_D)>>3;
//    RDMsg->KeyBoard.shift = (RDMsg->Key & KEY_PRESSED_OFFSET_SHIFT)>>4;
//    RDMsg->KeyBoard.ctrl  = (RDMsg->Key & KEY_PRESSED_OFFSET_CTRL)>>5;
//    RDMsg->KeyBoard.q     = (RDMsg->Key & KEY_PRESSED_OFFSET_Q)>>6;
//    RDMsg->KeyBoard.e     = (RDMsg->Key & KEY_PRESSED_OFFSET_E)>>7;
//    RDMsg->KeyBoard.r     = (RDMsg->Key & KEY_PRESSED_OFFSET_R)>>8;
//    RDMsg->KeyBoard.f     = (RDMsg->Key & KEY_PRESSED_OFFSET_F)>>9;
//    RDMsg->KeyBoard.g     = (RDMsg->Key & KEY_PRESSED_OFFSET_G)>>10;
//    RDMsg->KeyBoard.z     = (RDMsg->Key & KEY_PRESSED_OFFSET_Z)>>11;
//    RDMsg->KeyBoard.x     = (RDMsg->Key & KEY_PRESSED_OFFSET_X)>>12;
//    RDMsg->KeyBoard.c     = (RDMsg->Key & KEY_PRESSED_OFFSET_C)>>13;
//    RDMsg->KeyBoard.v     = (RDMsg->Key & KEY_PRESSED_OFFSET_V)>>14;
//    RDMsg->KeyBoard.b     = (RDMsg->Key & KEY_PRESSED_OFFSET_B)>>15; 
}

/**
  * @brief  ��ȡ��̨��Ϣ����
  * @param  ����ָ��ṹ��
  * @retval void
  * @attention 
  */
void Msg_HolderDataProcess(Holder_Data_T *HolderMsg)
{
	/* �����ǻ�ȡ������ԽǶ� */
//    get_eular(eular);
//    CircleContinue(&mc_imu_yaw, (eular[2] + 180.0f)*10.0f, 1800);  //ƫ��Yaw�Ƕ�
//    CircleContinue(&mc_imu_pitch, (eular[0] + 180.0f)*10.0f, 1800);  //����Pitch�Ƕ�
//	  CircleContinue(&mc_imu_pitch, (eular[1] + 180.0f)*10.0f, 1800);   
//    HolderMsg->angle[0] = mc_imu_pitch.Angle;// + mc_imu_pitch.Circle * 3600;
//    HolderMsg->angle[2] = mc_imu_yaw.Angle + mc_imu_yaw.Circle * 3600;	
    
	/* �����ǻ�ȡ���ٶ� */	
//    get_raw_gyo(gyro);
    HolderMsg->gyro[2] = gyro[2]; //Yaw���ٶ�
    HolderMsg->gyro[1] = gyro[1]; //Pitch���ٶ�
    HolderMsg->gyro[0] = gyro[0]; //Roll���ٶ�
}

/**
  * @brief  ��ȡ��̨��Ϣ����
  * @param  ����ָ��ṹ��
  * @retval void
  * @attention 
  */
void Msg_ChassisDataProcess(Chassis_Data_T *ChassisMsg)
{
	/* �����ǻ�ȡ������ԽǶ� */
    get_eular_chassis(eular);
    CircleContinue(&mc_imu_yaw_chassis, (eular[2] + 180.0f)*10.0f, 1800);  //ƫ��Yaw�Ƕ�
//	  CircleContinue(&mc_imu_pitch, (eular[1] + 180.0f)*10.0f, 1800);   
//    ChassisMsg->angle[0] = mc_imu_pitch.Angle;// + mc_imu_pitch.Circle * 3600;
    ChassisMsg->angle[2] = mc_imu_yaw_chassis.Angle + mc_imu_yaw_chassis.Circle * 3600;	
    
	/* �����ǻ�ȡ���ٶ� */	
    get_raw_gyo_chassis(gyro);
    ChassisMsg->gyro[2] = gyro[2]; //Yaw���ٶ�
//    ChassisMsg->gyro[1] = gyro[1]; //Pitch���ٶ�
//    ChassisMsg->gyro[0] = gyro[0]; //Roll���ٶ�
}

/**
  * @brief  ��ȡ����������
  * @param  ����ָ��ṹ��
  * @retval void
  * @attention 
  */
void getIMUEular(float *eular_tmp)
{
    uint8_t i;
    for(i=0; i<3; i++)
    {
        eular_tmp[i] = eular[i];
    }
}

void DownControl(void)
{
		uint8_t message[8] ={0};
		dir_angle = GetDirAngle_Shoot();
		vision_x = vData_x_speed*10000;
		vision_y = vData_y_speed*10000;
		vision_yaw1 = vData_yaw_speed*10000;
		message[0] = (uint8_t)(dir_angle >>8);
		message[1] = (uint8_t)(dir_angle);
		message[2] = (uint8_t)(vision_x >> 8);
		message[3] = (uint8_t)(vision_x);
		message[4] = (uint8_t)(vision_y >> 8);
		message[5] = (uint8_t)(vision_y);
		message[6] = (uint8_t)(vision_yaw1 >> 8);
		message[7] = (uint8_t)(vision_yaw1);
		CAN2_Transmit(0x201,message);
}

uint8_t change_bit(uint8_t val, uint8_t start, uint8_t end, uint8_t new_val) 
{
    uint32_t mask = (1 << (end - start + 1)) - 1; // ��������
    mask <<= start; // ��������������ȷλ��
    val &= ~mask; // ��val����Ӧλ����
    new_val <<= start; // ����ֵ��������ȷλ��
    return val | new_val; // ��ϸ��ĺ��λ
}

uint16_t remain_energy,down_heat;
uint16_t power_now;
uint8_t vision_message;
void DownControl2(void)
{
		uint8_t message[8] ={0};
		remain_energy = JUDGE_u16GetRemainEnergy();
		down_heat = JUDGE_u16GetRemoteHeat17(2);
		power_now = JUDGE_f32GetChassisPower() * 100;
		robot_ID = (uint8_t)JUDGE_u8GetRobotId();
		navigate = vData.recv_data.isNavigating;
		spining = vData.recv_data.isSpining;
		firesuggestion = vData.recv_data.firesuggestion;
		game_flag = JUDGE_u8game_progress();
		if(navigate == 0x01)
			vision_message = change_bit(vision_message,0,0,0x01);
		else 
			vision_message = change_bit(vision_message,0,0,0x00);
		if(spining == 0x01)
			vision_message = change_bit(vision_message,1,1,0x01);
		else 
			vision_message = change_bit(vision_message,1,1,0x00);
		if(firesuggestion == 0x01)
			vision_message = change_bit(vision_message,2,2,0x01);
		else 
			vision_message = change_bit(vision_message,2,2,0x00);
		if(game_flag == 4)
			vision_message = change_bit(vision_message,3,3,0x01);
		else 
			vision_message = change_bit(vision_message,3,3,0x00);
		if(JUDGE_u16bullet_remaining_num_17mm() > 10) 
			vision_message = change_bit(vision_message,4,4,0x01);
		else 
			vision_message = change_bit(vision_message,4,4,0x00);//���ｫ�ĸ���ֵ�ͱ����ϳ�Ϊһ��uint8�����ݣ����ٴӶ�����Ϣ����ѹ��������
		message[0] = (uint8_t)(remain_energy >> 8);
		message[1] = (uint8_t)(remain_energy);
		message[2] = (uint8_t)(down_heat >> 8);
		message[3] = (uint8_t)(down_heat);
		message[4] = (uint8_t)(power_now >> 8);
		message[5] = (uint8_t)(power_now);
		message[6] = (uint8_t)(vision_message);
		message[7] = (uint8_t)(robot_ID);
		CAN2_Transmit(0x202,message);
}
