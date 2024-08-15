#ifndef _JUDGE_TX_H
#define _JUDGE_TX_H

#include "usart.h"
#include <stdint.h>
/*  robotID 
    Red     hero1 engineer2 infantry3/4/5 air6 sentry7 radar9
    Blue    hero101 engineer102 infantry103/104/105 air106 sentry107 radar109
    clientID
    Red     hero0x0101 engineer0x0102 infantry0x0103/0x0104/0x0105 air0x0106
    Blue    hero0x0165 engineer0x0166 infantry0x0167/0x0168/0x0169 air0x016A
 */
#define RED1_robotID 1
#define RED2_robotID 2
#define RED3_robotID 3
#define RED4_robotID 4
#define RED5_robotID 5
#define RED6_robotID 6
#define RED7_robotID 7
#define RED9_robotID 9

#define BLUE1_robotID 101
#define BLUE2_robotID 102
#define BLUE3_robotID 103
#define BLUE4_robotID 104
#define BLUE5_robotID 105
#define BLUE6_robotID 106
#define BLUE7_robotID 107
#define BLUE9_robotID 109

#define RED1_clientID 0x0101
#define RED2_clientID 0x0102
#define RED3_clientID 0x0103
#define RED4_clientID 0x0104
#define RED5_clientID 0x0105
#define RED6_clientID 0x0106

#define BLUE1_clientID 0x0165
#define BLUE2_clientID 0x0166
#define BLUE3_clientID 0x0167
#define BLUE4_clientID 0x0168
#define BLUE5_clientID 0x0169
#define BLUE6_clientID 0x016A

typedef __packed struct
{
    uint16_t data_comment_ID;
    uint16_t sender_ID;
    uint16_t client_ID;
} ext_client_graphic_basic_t;

typedef __packed struct
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye : 3;
    uint32_t graphic_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t start_angle : 9;
    uint32_t end_angle : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t radius : 10;
    uint32_t end_x : 11;
    uint32_t end_y : 11;
} ext_client_graphic_draw_t;

typedef __packed struct 
{
    ext_client_graphic_basic_t ext_client_graphic_basic;
    ext_client_graphic_draw_t ext_client_graphic_draw;
    uint8_t text[30];
} ext_client_graphic_drawtext_t;

typedef __packed struct
{
    ext_client_graphic_basic_t ext_client_graphic_basic;
    ext_client_graphic_draw_t ext_client_graphic_draw[7];
} ext_client_graphic_drawseven_t;

typedef __packed struct
{
    ext_client_graphic_basic_t ext_client_graphic_basic;
    uint8_t operate_type;
    uint8_t layer;
} ext_client_graphic_drawdelete_t;

typedef enum
{
    _Lid_St,
    _Spin_Mode,
    _SuperCap_Data,
    //_HolderPitch_Data,
    _BackGround1,
    _BackGround2_SuperCap_Line,
	_Visionz_Data,
	_Vision_State
}judge_txpoll_e;

typedef __packed struct
{
		ext_client_graphic_basic_t ext_client_graphic_basic;
		uint32_t sentry_cmd; 
}sentry_cmd_t;

void JudgeUI_LidState(uint8_t robot_id, uint8_t rate_lv);
void JudgeUI_SpinMode(uint8_t robot_id, uint8_t state);
//void JudgeUI_PitchData(uint8_t robot_id);
//void JudgeUI_CapData(uint8_t robot_id);
void JudgeUI_SendBKG1(uint16_t sender_ID, uint8_t operate_tpye);
//void JudgeUI_BK2_CapLine(uint8_t robot_id);
//void JudgeUI_VisionzData(uint8_t robot_id);
void JudgeUI_Vision_State(uint8_t robot_id, uint8_t state);
void JudgeUI_SentryControl(uint16_t sender_ID,uint32_t sentry_cmd);
#endif
