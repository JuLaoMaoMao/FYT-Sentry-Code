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
    uint8_t *data_addr = p_frame + FRAME_HEADER_LEN + CMD_ID_LEN; /* 数据帧头地址 */

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

/********************裁判数据辅助判断函数***************************/

/**
  * @brief  读取瞬时功率
  * @param  void
  * @retval 实时功率值
  * @attention  
  */
float JUDGE_f32GetChassisPower(void)
{
    return (judge_rxmsg.ext_power_heat_data.chassis_power);
}

/**
  * @brief  读取功率限制
  * @param  void
  * @retval 功率限制
  * @attention  
  */
uint16_t JUDGE_u16GetChassisPowerLimit(void)
{
    return (judge_rxmsg.ext_game_robot_status.chassis_power_limit);
}

/**
  * @brief  读取剩余焦耳能量
  * @param  void
  * @retval 剩余缓冲焦耳能量(最大200)
  * @attention  
  */
uint16_t JUDGE_u16GetRemainEnergy(void)
{
    return (judge_rxmsg.ext_power_heat_data.chassis_power_buffer);
}

/**
  * @brief  读取当前等级
  * @param  void
  * @retval 当前等级
  * @attention  
  */
uint8_t JUDGE_u8GetRobotLevel(void)
{
    return (judge_rxmsg.ext_game_robot_status.robot_level);
}

/**
  * @brief  读取枪口热量
  * @param  枪口ID
  * @retval 17mm枪口热量
  * @attention  实时热量
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
  * @brief  读取射速
  * @param  void
  * @retval 17mm射速
  * @attention  实时热量
  */
float JUDGE_fGetSpeedHeat17(void)
{
    return judge_rxmsg.ext_shoot_data.bullet_speed;
}

/**
  * @brief  读取射速限制
  * @param  void
  * @retval 17mm射速限制
  * @attention 
  */
//uint16_t JUDGE_u16GetSpeedHeat17Limit(void)
//{
//    return judge_rxmsg.ext_game_robot_status.shooter_speed_limit;
//}

/**
  * @brief  读取枪口热量上限
  * @param  void
  * @retval 当前等级17mm热量上限
  * @attention  
  */
uint16_t JUDGE_u16GetHeatLimit(void)
{
    return (judge_rxmsg.ext_game_robot_status.shooter_cooling_limit);
}

/**
  * @brief  读取枪口热量冷却速率
  * @param  void
  * @retval 当前等级17mm热量冷却速率
  * @attention  
  */
uint16_t JUDGE_u16GetHeatRate(void)
{
    return (judge_rxmsg.ext_game_robot_status.shooter_cooling_rate);
}

/**
  * @brief  获取机器人ID
  * @param  void
  * @retval 机器人ID
  * @attention 
  */
uint8_t JUDGE_u8GetRobotId(void)
{
    return (judge_rxmsg.ext_game_robot_status.robot_id);
}

/**
  * @brief  底盘电压
  * @param  void
  * @retval 底盘电压
  * @attention 
  */
uint16_t JUDGE_u16GetChassisVolt(void)
{
    return (judge_rxmsg.ext_power_heat_data.chassis_volt);
}

/**
  * @brief  受攻击判断
  * @param  void
  * @retval 是否受攻击
  * @attention 
  */
uint8_t JUDGE_u8robot_hurt(void)
{
    return (judge_rxmsg.ext_robot_hurt.hurt_type);
}

/**
  * @brief  读取血量
  * @param  void
  * @retval 哨兵血量
  * @attention 
  */
uint16_t JUDGE_u16robot_HP()
{
	uint16_t Sentry_HP=400;
	Sentry_HP  = judge_rxmsg.ext_game_robot_status.remain_HP;
	return(Sentry_HP);
}

/**
  * @brief  剩余时间
  * @param  void
  * @retval 剩余时间 
  * @attention 
  */
uint16_t JUDGE_u8remaining_time()
{
    return (judge_rxmsg.ext_game_status.stage_remain_time);
}

/**
  * @brief  前哨站血量
  * @param  void
  * @retval 前哨站血量 
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
  * @brief  前哨站血量
  * @param  void
  * @retval 前哨站血量 
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
  * @brief  基地血量
  * @param  void
  * @retval 基地血量 
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
  * @brief  比赛阶段
  * @param  void
  * @retval 比赛阶段 
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
  * @brief  剩余弹量
  * @param  void
  * @retval 剩余弹量 
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
  * @brief  云台手控制
  * @param  void
  * @retval 云台手控制 
  * @attention 
  */
uint8_t JUDGE_u8_robot_command(void)
{
	return(judge_rxmsg.ext_robot_command.commd_keyboard);
//	 return(judge_rxmsg.ext_robot_command.target_robot_ID);
}

/**
  * @brief  比赛剩余时间
  * @param  void
  * @retval 比赛剩余时间 
  * @attention 
  */
uint16_t JUDGE_u16remaining_time(void)
{
	return(judge_rxmsg.ext_game_status.stage_remain_time);
}

/**
  * @brief  比赛阶段
  * @param  void
  * @retval 比赛阶段 
  * @attention 
  */
uint8_t JUDGE_u8_game_progress(void)
{
	return(judge_rxmsg.ext_game_status.game_progress);
}

/**
  * @brief  射频
  * @param  测速ID
  * @retval 射频 
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
  * @brief  射速
  * @param  测速ID
  * @retval 射速 
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
  * @brief  装甲板击打情况
  * @param  void
  * @retval 装甲板击打情况 
  * @attention 
  */
uint8_t JUDGE_u8_hurt_id(void)
{
	return(judge_rxmsg.ext_robot_hurt.armor_id);
}

/**
  * @brief  读取当前经济
  * @param  uint16_t
  * @retval 功率限制
  * @attention  
  */
uint16_t JUDGE_u16Remaining_gold(void)
{
    return (judge_rxmsg.ext_bullet_remaining.coin_remaining_num);
}

/**
  * @brief  读取是否在补给区
  * @param  uint8_t
  * @retval 补给区ID
  * @attention  
  */
uint8_t JUDGE_u8intakearea_id(void)
{
    return (judge_rxmsg.ext_area_data.area_data >> 12 & 0x01);
}

/**
  * @brief  读取是否在哨兵巡逻区
  * @param  uint8_t
  * @retval 
  * @attention  
  */
uint8_t JUDGE_u8patrolarea_id(void)
{
    return (judge_rxmsg.ext_area_data.area_data >> 13 & 0x01);
}
