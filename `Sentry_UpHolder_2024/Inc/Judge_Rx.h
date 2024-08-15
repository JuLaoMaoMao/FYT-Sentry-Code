#ifndef _JUDGE_RX_H
#define _JUDGE_RX_H

#include "stdint.h"

typedef enum
{
    game_status_id = 0x0001,
    game_result_id = 0x0002,
    game_robot_HP_id = 0x0003,
    dart_status_id = 0x0004,
    event_data_id = 0x0101,
    supply_projectile_action_id = 0x0102,
    dart_remaining_time = 0x0105,
    game_robot_status = 0x0201,
    power_heat_data_id = 0x0202,
    game_game_robot_pos_id = 0x0203,
    buff_id = 0x0204,
    air_robot_energy = 0x0205,
    robot_hurt_id = 0x0206,
    shoot_data_id = 0x0207,
    bullet_remaining_id = 0x0208,
		area_data_id = 0x0209,
    dart_client_cmd_id = 0x020A,
    robot_command_id = 0x0303
} cmd_id_e;

typedef __packed struct
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_status_t;

typedef __packed struct
{
    uint8_t winner;
} ext_game_result_t;

typedef __packed struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

typedef __packed struct
{
    uint8_t dart_belong;
    uint16_t stage_remaining_time;
} ext_dart_status_t;

typedef __packed struct
{
    uint32_t event_type;
} ext_event_data_t;

typedef __packed struct
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef __packed struct
{
    uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

typedef __packed struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_cooling_rate;
    uint16_t shooter_cooling_limit;
    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;

typedef __packed struct
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

typedef __packed struct
{
    float x;
    float y;
//    float z;
    float yaw;
} ext_game_robot_pos_t;

typedef __packed struct
{
    uint8_t power_rune_buff;
} ext_buff_t;

typedef __packed struct
{
    uint8_t attack_time;
} air_robot_energy_t;

typedef __packed struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __packed struct
{
		float target_position_x;
		float target_position_y;
		uint8_t commd_keyboard;
		uint8_t target_robot_ID;
		uint16_t cmd_source;
} ext_robot_command_t;

typedef __packed struct
{
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;

typedef __packed struct
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

typedef __packed struct
{
    uint8_t dart_launch_opening_status;
    uint8_t dart_attack_target;
    uint16_t target_change_time;
    uint8_t first_dart_speed;
    uint8_t second_dart_speed;
    uint8_t third_dart_speed;
    uint8_t fourth_dart_speed;
    uint16_t last_dart_launch_time;
    uint16_t operate_launcch_cmd_time;
} ext_dart_client_cmd_t;

typedef  __packed struct
{
		uint32_t area_data;
}ext_area_data_t;
typedef struct
{
    ext_game_status_t ext_game_status;
    ext_game_result_t ext_game_result;
    ext_game_robot_HP_t ext_game_robot_HP;
    ext_dart_status_t ext_dart_status;
    ext_event_data_t ext_event_data;
    ext_supply_projectile_action_t ext_supply_projectile_action;
    ext_dart_remaining_time_t ext_dart_remaining_time;
    ext_game_robot_status_t ext_game_robot_status;
    ext_power_heat_data_t ext_power_heat_data;
    ext_game_robot_pos_t ext_game_robot_pos;
    ext_buff_t ext_buff;
    air_robot_energy_t air_robot_energy;
    ext_robot_hurt_t ext_robot_hurt;
    ext_robot_command_t ext_robot_command;
    ext_shoot_data_t ext_shoot_data;
    ext_bullet_remaining_t ext_bullet_remaining;
    ext_dart_client_cmd_t ext_dart_client_cmd;
		ext_area_data_t ext_area_data;
} judge_rxmsg_t;

void judge_rx_handler(uint8_t *p_frame);

/************************裁判系统辅助函数***********************/
float JUDGE_f32GetChassisPower(void);         /*< 读取底盘功率*/
uint16_t JUDGE_u16GetChassisPowerLimit(void); /*< 读取底盘功率上限*/
uint16_t JUDGE_u16GetRemainEnergy(void);      /*< 读取剩余焦耳能量*/
uint8_t JUDGE_u8GetRobotLevel(void);          /*< 读取机器人等级*/
uint16_t JUDGE_u16GetRemoteHeat17(uint8_t id);      /*< 读取17mm枪口热量*/
float JUDGE_fGetSpeedHeat17(void);            /*< 读取17mm射速*/
uint16_t JUDGE_u16GetSpeedHeat17Limit(void);  /*< 读取17mm射速限制*/
uint16_t JUDGE_u16GetHeatLimit(void);         /*< 读取热量上限*/
uint16_t JUDGE_u16GetShootCold(void);         /*< 读取冷却秒速*/
uint8_t JUDGE_u8GetRobotId(void);             /*< 读取机器人ID*/
uint16_t JUDGE_u16GetHeatRate(void);          /*< 读取枪口热量冷却速率*/
uint16_t JUDGE_u16GetChassisVolt(void);       /*< 读取底盘电压*/
uint8_t JUDGE_u8robot_hurt(void);             /*< 读取受伤状态*/
uint16_t JUDGE_u16robot_HP(void);             /*< 读取哨兵血量*/
uint16_t JUDGE_u16remaining_time(void);         /*< 读取剩余时间*/
uint16_t  JUDGE_u16outpost_HP(void);          /*< 读取前哨站血量*/
uint16_t JUDGE_u16enemyoutpost_HP(void);      /*< 读取敌方前哨站血量*/
uint16_t  JUDGE_u16base_HP(void);             /*< 读取基地血量*/
uint8_t JUDGE_u8game_progress(void);          /*< 读取比赛阶段*/
uint16_t JUDGE_u16bullet_remaining_num_17mm(void);/*< 读取剩余弹量*/
uint8_t JUDGE_u8_robot_command(void);          /*< 云台手控制*/ 
uint8_t JUDGE_u8_game_progress(void);          /*<获取比赛状态*/     /*< 读取云台手按键 */
uint8_t JUDGE_u8_shoot_fre(uint8_t shoot_id);  /*< 射频 */
float JUDGE_u8_shoot_speed(uint8_t shoot_id);		/*< 射速 */
uint8_t JUDGE_u8_hurt_id(void);   /*< 被击打装甲板ID */
uint16_t JUDGE_u16Remaining_gold(void);					/*读取当前经济*/
uint8_t JUDGE_u8intakearea_id(void);						/*读取是否在补给区*/
uint8_t JUDGE_u8patrolarea_id(void);            /*读取是否在哨兵巡逻区*/
#endif
