#include "Judge_Rx.h"
#include "dma_unpack.h"
#include "string.h"
#include "Vision.h"
judge_rxmsg_t judge_rxmsg;
uint16_t cmd_id_test;
void judge_rx_handler(uint8_t *p_frame)
{
    frame_header_t *p_header = (frame_header_t *)p_frame;

    uint16_t data_length = p_header->data_len;
    uint16_t cmd_id = *(uint16_t *)(p_frame + FRAME_HEADER_LEN);
    uint8_t *data_addr = p_frame + FRAME_HEADER_LEN + CMD_ID_LEN; /* ����֡ͷ��ַ */

    memcpy(p_header, p_frame, FRAME_HEADER_LEN);

    switch (cmd_id)
    {
    case game_status_id:
        memcpy(&judge_rxmsg.ext_game_status, data_addr, data_length);
        break;

    case game_result_id:
        memcpy(&judge_rxmsg.ext_game_result, data_addr, data_length);
        break;

    case game_robot_HP_id:
        memcpy(&judge_rxmsg.ext_game_robot_HP, data_addr, data_length);
        break;

    case dart_status_id:
        memcpy(&judge_rxmsg.ext_dart_status, data_addr, data_length);
        break;

    case event_data_id:
        memcpy(&judge_rxmsg.ext_event_data, data_addr, data_length);
        break;

    case supply_projectile_action_id:
        memcpy(&judge_rxmsg.ext_supply_projectile_action, data_addr, data_length);
        break;

    case dart_remaining_time:
        memcpy(&judge_rxmsg.ext_dart_remaining_time, data_addr, data_length);
        break;

    case game_robot_status:
        memcpy(&judge_rxmsg.ext_game_robot_status, data_addr, data_length);
        break;

    case power_heat_data_id:
        memcpy(&judge_rxmsg.ext_power_heat_data, data_addr, data_length);
        break;

    case game_game_robot_pos_id:
        memcpy(&judge_rxmsg.ext_game_robot_pos, data_addr, data_length);
        break;

    case buff_id:
        memcpy(&judge_rxmsg.ext_buff, data_addr, data_length);
        break;

    case air_robot_energy:
        memcpy(&judge_rxmsg.air_robot_energy, data_addr, data_length);
        break;

    case robot_hurt_id:
        memcpy(&judge_rxmsg.ext_robot_hurt, data_addr, data_length);
        break;

    case shoot_data_id:
        memcpy(&judge_rxmsg.ext_shoot_data, data_addr, data_length);
        break;

    case bullet_remaining_id:
        memcpy(&judge_rxmsg.ext_bullet_remaining, data_addr, data_length);
        break;

    case dart_client_cmd_id:
        memcpy(&judge_rxmsg.ext_dart_client_cmd, data_addr, data_length);
        break;
    
    case robot_command_id:
        memcpy(&judge_rxmsg.ext_robot_command, data_addr, data_length);
        break;
		
		case area_data_id:
				memcpy(&judge_rxmsg.ext_area_data, data_addr, data_length);
				break;
		
    default:
        break;
    }
}

/********************�������ݸ����жϺ���***************************/

/**
  * @brief  ��ȡ˲ʱ����
  * @param  void
  * @retval ʵʱ����ֵ
  * @attention  
  */
float JUDGE_f32GetChassisPower(void)
{
    return (judge_rxmsg.ext_power_heat_data.chassis_power);
}

/**
  * @brief  ��ȡ��������
  * @param  void
  * @retval ��������
  * @attention  
  */
uint16_t JUDGE_u16GetChassisPowerLimit(void)
{
    return (judge_rxmsg.ext_game_robot_status.chassis_power_limit);
}

/**
  * @brief  ��ȡʣ�ཹ������
  * @param  void
  * @retval ʣ�໺�役������(���200)
  * @attention  
  */
uint16_t JUDGE_u16GetRemainEnergy(void)
{
    return (judge_rxmsg.ext_power_heat_data.chassis_power_buffer);
}

/**
  * @brief  ��ȡ��ǰ�ȼ�
  * @param  void
  * @retval ��ǰ�ȼ�
  * @attention  
  */
uint8_t JUDGE_u8GetRobotLevel(void)
{
    return (judge_rxmsg.ext_game_robot_status.robot_level);
}

/**
  * @brief  ��ȡǹ������
  * @param  ǹ��ID
  * @retval 17mmǹ������
  * @attention  ʵʱ����
  */
uint16_t JUDGE_u16GetRemoteHeat17(uint8_t id)
{
	uint16_t heat;
	switch(id)
	{
		case 1:
				heat=judge_rxmsg.ext_power_heat_data.shooter_id1_17mm_cooling_heat;break;
		case 2:
			  heat=judge_rxmsg.ext_power_heat_data.shooter_id2_17mm_cooling_heat;break;
	}
	return(heat);
}

/**
  * @brief  ��ȡ����
  * @param  void
  * @retval 17mm����
  * @attention  ʵʱ����
  */
float JUDGE_fGetSpeedHeat17(void)
{
    return judge_rxmsg.ext_shoot_data.bullet_speed;
}

/**
  * @brief  ��ȡ��������
  * @param  void
  * @retval 17mm��������
  * @attention 
  */
//uint16_t JUDGE_u16GetSpeedHeat17Limit(void)
//{
//    return judge_rxmsg.ext_game_robot_status.shooter_speed_limit;
//}

/**
  * @brief  ��ȡǹ����������
  * @param  void
  * @retval ��ǰ�ȼ�17mm��������
  * @attention  
  */
uint16_t JUDGE_u16GetHeatLimit(void)
{
    return (judge_rxmsg.ext_game_robot_status.shooter_cooling_limit);
}

/**
  * @brief  ��ȡǹ��������ȴ����
  * @param  void
  * @retval ��ǰ�ȼ�17mm������ȴ����
  * @attention  
  */
uint16_t JUDGE_u16GetHeatRate(void)
{
    return (judge_rxmsg.ext_game_robot_status.shooter_cooling_rate);
}

/**
  * @brief  ��ȡ������ID
  * @param  void
  * @retval ������ID
  * @attention 
  */
uint8_t JUDGE_u8GetRobotId(void)
{
    return (judge_rxmsg.ext_game_robot_status.robot_id);
}

/**
  * @brief  ���̵�ѹ
  * @param  void
  * @retval ���̵�ѹ
  * @attention 
  */
uint16_t JUDGE_u16GetChassisVolt(void)
{
    return (judge_rxmsg.ext_power_heat_data.chassis_volt);
}

/**
  * @brief  �ܹ����ж�
  * @param  void
  * @retval �Ƿ��ܹ���
  * @attention 
  */
uint8_t JUDGE_u8robot_hurt(void)
{
    return (judge_rxmsg.ext_robot_hurt.hurt_type);
}

/**
  * @brief  ��ȡѪ��
  * @param  void
  * @retval �ڱ�Ѫ��
  * @attention 
  */
uint16_t JUDGE_u16robot_HP()
{
	uint16_t Sentry_HP=400;
	Sentry_HP  = judge_rxmsg.ext_game_robot_status.remain_HP;
	return(Sentry_HP);
}

/**
  * @brief  ʣ��ʱ��
  * @param  void
  * @retval ʣ��ʱ�� 
  * @attention 
  */
uint16_t JUDGE_u8remaining_time()
{
    return (judge_rxmsg.ext_game_status.stage_remain_time);
}

/**
  * @brief  ǰ��վѪ��
  * @param  void
  * @retval ǰ��վѪ�� 
  * @attention 
  */
uint16_t JUDGE_u16outpost_HP()
{
	  switch(Robot_ID)
	  {
	  	case 7:
				return (judge_rxmsg.ext_game_robot_HP.red_outpost_HP);
//				return 1500;
//	  		 Outpost_HP=judge_rxmsg.ext_game_robot_HP.red_outpost_HP;break;
	  	case 107:
				return (judge_rxmsg.ext_game_robot_HP.blue_outpost_HP);
//				return 0;
			case 0 :
				return 666;
//	  		 Outpost_HP=judge_rxmsg.ext_game_robot_HP.blue_outpost_HP;
	  }
}

/**
  * @brief  ǰ��վѪ��
  * @param  void
  * @retval ǰ��վѪ�� 
  * @attention 
  */
uint16_t JUDGE_u16enemyoutpost_HP()
{
	  switch(Robot_ID)
	  {
	  	case 7:
				return (judge_rxmsg.ext_game_robot_HP.blue_outpost_HP);
//				return 1500;
//	  		 Outpost_HP=judge_rxmsg.ext_game_robot_HP.red_outpost_HP;break;
	  	case 107:
				return (judge_rxmsg.ext_game_robot_HP.red_outpost_HP);
//				return 0;
			case 0 :
				return 666;
//	  		 Outpost_HP=judge_rxmsg.ext_game_robot_HP.blue_outpost_HP;
	  }
}

/**
  * @brief  ����Ѫ��
  * @param  void
  * @retval ����Ѫ�� 
  * @attention 
  */
uint16_t JUDGE_u16base_HP()
{
	  uint16_t Base_HP=2500;
	  switch(judge_rxmsg.ext_game_robot_status.robot_id)
	  {
	  	case 7:
	  		 Base_HP=judge_rxmsg.ext_game_robot_HP.red_base_HP;break;
	  	case 107:
	  		 Base_HP=judge_rxmsg.ext_game_robot_HP.blue_base_HP;break;
	  }
	  return(Base_HP);
}

/**
  * @brief  �����׶�
  * @param  void
  * @retval �����׶� 
  * @attention 
  */
uint8_t JUDGE_u8game_progress()
{
		#ifdef DEBUG
			return 4;
		#endif
			return(judge_rxmsg.ext_game_status.game_progress);
//	  return 4;
}

/**
  * @brief  ʣ�൯��
  * @param  void
  * @retval ʣ�൯�� 
  * @attention 
  */
uint16_t JUDGE_u16bullet_remaining_num_17mm()
{
	#ifdef DEBUG
			return 300;
		#endif
	return(judge_rxmsg.ext_bullet_remaining.bullet_remaining_num_17mm);
}

/**
  * @brief  ��̨�ֿ���
  * @param  void
  * @retval ��̨�ֿ��� 
  * @attention 
  */
uint8_t JUDGE_u8_robot_command(void)
{
	return(judge_rxmsg.ext_robot_command.commd_keyboard);
//	 return(judge_rxmsg.ext_robot_command.target_robot_ID);
}

/**
  * @brief  ����ʣ��ʱ��
  * @param  void
  * @retval ����ʣ��ʱ�� 
  * @attention 
  */
uint16_t JUDGE_u16remaining_time(void)
{
	return(judge_rxmsg.ext_game_status.stage_remain_time);
}

/**
  * @brief  �����׶�
  * @param  void
  * @retval �����׶� 
  * @attention 
  */
uint8_t JUDGE_u8_game_progress(void)
{
	return(judge_rxmsg.ext_game_status.game_progress);
}

/**
  * @brief  ��Ƶ
  * @param  ����ID
  * @retval ��Ƶ 
  * @attention 
  */
uint8_t JUDGE_u8_shoot_fre(uint8_t shoot_id)
{
	uint8_t frequence;
	if(judge_rxmsg.ext_shoot_data.shooter_id == shoot_id)
	{
			frequence = judge_rxmsg.ext_shoot_data.bullet_freq;
	}
	else
	{
			frequence = 0;
	}
	return frequence;
}

/**
  * @brief  ����
  * @param  ����ID
  * @retval ���� 
  * @attention 
  */
float JUDGE_u8_shoot_speed(uint8_t shoot_id)
{
	float speed;
	if(judge_rxmsg.ext_shoot_data.shooter_id == shoot_id)
	{
			speed = judge_rxmsg.ext_shoot_data.bullet_speed;
	}
	else
	{
			speed = 0;
	}
	return speed;
}

/**
  * @brief  װ�װ�������
  * @param  void
  * @retval װ�װ������� 
  * @attention 
  */
uint8_t JUDGE_u8_hurt_id(void)
{
	return(judge_rxmsg.ext_robot_hurt.armor_id);
}

/**
  * @brief  ��ȡ��ǰ����
  * @param  uint16_t
  * @retval ��������
  * @attention  
  */
uint16_t JUDGE_u16Remaining_gold(void)
{
    return (judge_rxmsg.ext_bullet_remaining.coin_remaining_num);
}

/**
  * @brief  ��ȡ�Ƿ��ڲ�����
  * @param  uint8_t
  * @retval ������ID
  * @attention  
  */
uint8_t JUDGE_u8intakearea_id(void)
{
    return (judge_rxmsg.ext_area_data.area_data >> 12 & 0x01);
}

/**
  * @brief  ��ȡ�Ƿ����ڱ�Ѳ����
  * @param  uint8_t
  * @retval 
  * @attention  
  */
uint8_t JUDGE_u8patrolarea_id(void)
{
    return (judge_rxmsg.ext_area_data.area_data >> 13 & 0x01);
}
