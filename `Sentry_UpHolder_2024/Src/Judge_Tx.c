/**
  ******************************************************************************
  * @file    
  * @author  v5.0hxl
  * @brief
  * @date     
  ******************************************************************************
  * @attention
  *
  * Copyright (c) CSU_RM_FYT.
  * All rights reserved.
  *
  * This software component is licensed by SY under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* includes ------------------------------------------------------------------*/
#include "Judge_Tx.h"
#include "dma_unpack.h"
#include "string.h"
#include "Vision.h"
/* typedef -------------------------------------------------------------------*/
/* define --------------------------------------------------------------------*/
/* variables -----------------------------------------------------------------*/
uint8_t StrTmp[30] = {0}; /*字符型数字输出*/
ext_client_graphic_drawseven_t ext_client_graphic_drawseven;
ext_client_graphic_drawtext_t judge_txmsg;
sentry_cmd_t sentry_control_txmsg;


/* function ------------------------------------------------------------------*/
/*
    数字输出,显示不出来,暂保留
	ext_client_graphic_draw.ext_client_graphic_draw.radius = (numberint)&0x3FF;
    ext_client_graphic_draw.ext_client_graphic_draw.end_x = (numberint>>10)&0x7FF;
    ext_client_graphic_draw.ext_client_graphic_draw.end_y = (numberint>>21)&0x3FF;
*/
/**
  * @brief  获得对应接收客户端ID
  * @param  uint16_t sender_ID
  * @retval uint16_t client_ID
  * @attention 
  */
static uint16_t get_client_ID(uint16_t sender_ID)
{
    uint16_t client_ID;
    switch (sender_ID)
    {
    case RED1_robotID:
        client_ID = 0x0101;
        break;
    case RED2_robotID:
        client_ID = 0x0102;
        break;
    case RED3_robotID:
        client_ID = 0x0103;
        break;
    case RED4_robotID:
        client_ID = 0x0104;
        break;
    case RED5_robotID:
        client_ID = 0x0105;
        break;
    case RED6_robotID:
        client_ID = 0x0106;
        break;

    case BLUE1_robotID:
        client_ID = 0x0165;
        break;
    case BLUE2_robotID:
        client_ID = 0x0166;
        break;
    case BLUE3_robotID:
        client_ID = 0x0167;
        break;
    case BLUE4_robotID:
        client_ID = 0x0168;
        break;
    case BLUE5_robotID:
        client_ID = 0x0169;
        break;
    case BLUE6_robotID:
        client_ID = 0x016A;
        break;
    }
    return client_ID;
}

/**
  * @brief  发送弹仓盖数据
  * @param  uint16_t sender_ID发送端ID
  * @param  uint8_t Lid_State弹仓盖状态
  * @param  uint8_t operate_tpye对图像的操作
  * @retval void
  * @attention 
  */
void JudgeUI_SendLidSt(uint16_t sender_ID, uint8_t Lid_State, uint8_t operate_tpye)
{
    uint8_t i;
    judge_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0110;
    judge_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
    judge_txmsg.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    judge_txmsg.ext_client_graphic_draw.graphic_name[0] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[1] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '1';
    judge_txmsg.ext_client_graphic_draw.operate_tpye = operate_tpye;
    judge_txmsg.ext_client_graphic_draw.start_angle = 25;
    judge_txmsg.ext_client_graphic_draw.end_angle = 6;
    judge_txmsg.ext_client_graphic_draw.graphic_tpye = 7;
    judge_txmsg.ext_client_graphic_draw.layer = 9;
    judge_txmsg.ext_client_graphic_draw.color = 0;
    judge_txmsg.ext_client_graphic_draw.width = 5;
    judge_txmsg.ext_client_graphic_draw.start_x = 200;
    judge_txmsg.ext_client_graphic_draw.start_y = 650;
    judge_txmsg.ext_client_graphic_draw.radius = 0;
    judge_txmsg.ext_client_graphic_draw.end_x = 0;
    judge_txmsg.ext_client_graphic_draw.end_y = 0;
    for (i = 0; i < 30; i++)
    {
        judge_txmsg.text[i] = 0;
    }
    judge_txmsg.text[0] = 'L';
    judge_txmsg.text[1] = 'i';
    judge_txmsg.text[2] = 'd';
    judge_txmsg.text[3] = ':';
    judge_txmsg.text[4] = Lid_State;

    if (Lid_State == 'G') /*弹仓盖没开则为白色*/
        judge_txmsg.ext_client_graphic_draw.color = 8;

    /*数据封装*/
    data_packet_pack((0x0301), (uint8_t *)&judge_txmsg, sizeof(ext_client_graphic_drawtext_t), 0xA5);

    /*DMA发送*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}

/**
  * @brief  发送小陀螺状态
  * @param  uint16_t sender_ID发送端ID, 
  * @param  uint8_t spin_state小陀螺状态,
  * @param  uint8_t operate_tpye对图像的操作
  * @retval void
  * @attention 
  */
void JudgeUI_SendSpinMode(uint16_t sender_ID, uint8_t spin_state, uint8_t operate_tpye)
{
    int i;
    judge_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0110;
    judge_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
    judge_txmsg.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    judge_txmsg.ext_client_graphic_draw.graphic_name[0] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[1] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '2';
    judge_txmsg.ext_client_graphic_draw.operate_tpye = operate_tpye;
    judge_txmsg.ext_client_graphic_draw.start_angle = 25;
    judge_txmsg.ext_client_graphic_draw.end_angle = 6;
    judge_txmsg.ext_client_graphic_draw.graphic_tpye = 7;
    judge_txmsg.ext_client_graphic_draw.layer = 8;
    judge_txmsg.ext_client_graphic_draw.color = 0;
    judge_txmsg.ext_client_graphic_draw.width = 5;
    judge_txmsg.ext_client_graphic_draw.start_x = 200;
    judge_txmsg.ext_client_graphic_draw.start_y = 600;
    judge_txmsg.ext_client_graphic_draw.radius = 0;
    judge_txmsg.ext_client_graphic_draw.end_x = 0;
    judge_txmsg.ext_client_graphic_draw.end_y = 0;
    for (i = 0; i < 30; i++)
    {
        judge_txmsg.text[i] = 0;
    }
    judge_txmsg.text[0] = 'M';
    judge_txmsg.text[1] = 'o';
    judge_txmsg.text[2] = 'd';
    judge_txmsg.text[3] = 'e';
    judge_txmsg.text[4] = ':';
//    judge_txmsg.text[5] = spin_state;

    if (spin_state == 'f') /*跟随模式为白色*/
        judge_txmsg.ext_client_graphic_draw.color = 8;

    /*数据封装*/
    data_packet_pack((0x0301), (uint8_t *)&judge_txmsg, sizeof(ext_client_graphic_drawtext_t), 0xA5);

    /*DMA发送*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}

/**
  * @brief  发送打符状态
  * @param  uint16_t sender_ID发送端ID, 
  * @param  uint8_t vision_state打符状态,
  * @param  uint8_t operate_tpye对图像的操作
  * @retval void
  * @attention 
  */
void JudgeUI_SendVisionState(uint16_t sender_ID, uint8_t vision_state, uint8_t operate_tpye)
{
    int i;
    judge_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0110;
    judge_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
    judge_txmsg.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    judge_txmsg.ext_client_graphic_draw.graphic_name[0] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[1] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '5';
    judge_txmsg.ext_client_graphic_draw.operate_tpye = operate_tpye;
    judge_txmsg.ext_client_graphic_draw.start_angle = 25;
    judge_txmsg.ext_client_graphic_draw.end_angle = 6;
    judge_txmsg.ext_client_graphic_draw.graphic_tpye = 7;
    judge_txmsg.ext_client_graphic_draw.layer = 8;
    judge_txmsg.ext_client_graphic_draw.color = 0;
    judge_txmsg.ext_client_graphic_draw.width = 5;
    judge_txmsg.ext_client_graphic_draw.start_x = 200;
    judge_txmsg.ext_client_graphic_draw.start_y = 550;
    judge_txmsg.ext_client_graphic_draw.radius = 0;
    judge_txmsg.ext_client_graphic_draw.end_x = 0;
    judge_txmsg.ext_client_graphic_draw.end_y = 0;
    for (i = 0; i < 30; i++)
    {
        judge_txmsg.text[i] = 0;
    }
    judge_txmsg.text[0] = 'V';
    judge_txmsg.text[1] = 'i';
    judge_txmsg.text[2] = 's';
    judge_txmsg.text[3] = 'i';
    judge_txmsg.text[4] = ':';
    judge_txmsg.text[5] = vision_state;

    if (vision_state == 'A') /*自瞄模式为白色*/
        judge_txmsg.ext_client_graphic_draw.color = 8;

    /*数据封装*/
    data_packet_pack((0x0301), (uint8_t *)&judge_txmsg, sizeof(ext_client_graphic_drawtext_t), 0xA5);

    /*DMA发送*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}



/**
  * @brief  发送超级电容数据
  * @param  uint16_t sender_ID发送端ID,uint8_t *Str,float tmp,uint8_t operate_tpye对图像的操作
  * @retval void
  * @attention 
  */
void JudgeUI_SendSuperCapData(uint16_t sender_ID, uint8_t *Str, float tmp, uint8_t operate_tpye)
{
    uint8_t i;
    judge_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0110;
    judge_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
    judge_txmsg.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    judge_txmsg.ext_client_graphic_draw.graphic_name[0] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[1] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '3';
    judge_txmsg.ext_client_graphic_draw.operate_tpye = operate_tpye;
    judge_txmsg.ext_client_graphic_draw.graphic_tpye = 7;
    judge_txmsg.ext_client_graphic_draw.start_angle = 25;
    judge_txmsg.ext_client_graphic_draw.end_angle = 6;
    judge_txmsg.ext_client_graphic_draw.layer = 9;
    judge_txmsg.ext_client_graphic_draw.color = 0;
    judge_txmsg.ext_client_graphic_draw.width = 3;
    judge_txmsg.ext_client_graphic_draw.start_x = 1400;
    judge_txmsg.ext_client_graphic_draw.start_y = 700;
    judge_txmsg.ext_client_graphic_draw.radius = 0;
    judge_txmsg.ext_client_graphic_draw.end_x = 0;
    judge_txmsg.ext_client_graphic_draw.end_y = 0;
    for (i = 0; i < 30; i++)
        judge_txmsg.text[i] = 0;
    memcpy(judge_txmsg.text, Str, sizeof(Str) + 1);
    sprintf((char *)&StrTmp, "%4.2f", tmp);
    memcpy(judge_txmsg.text + sizeof(Str) + 1, StrTmp, sizeof(StrTmp));

    /*数据封装*/
    data_packet_pack(0x0301, (uint8_t *)&judge_txmsg, sizeof(judge_txmsg), 0xA5);
    /*DMA发送，清除字符串*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
    memset(&StrTmp, 0, sizeof(StrTmp));
}

///**
//  * @brief  发送Pitch俯仰角数据
//  * @param  uint16_t sender_ID发送端ID,uint8_t *Str,float tmp,uint8_t operate_tpye对图像的操作
//  * @retval void
//  * @attention 
//  */
//void JudgeUI_SendHolderPitchData(uint16_t sender_ID, uint8_t *Str, float tmp, uint8_t operate_tpye)
//{
//    uint8_t i;
//    judge_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0110;
//    judge_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
//    judge_txmsg.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
//    judge_txmsg.ext_client_graphic_draw.graphic_name[0] = '0';
//    judge_txmsg.ext_client_graphic_draw.graphic_name[1] = '0';
//    judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '4';
//    judge_txmsg.ext_client_graphic_draw.operate_tpye = operate_tpye;
//    judge_txmsg.ext_client_graphic_draw.graphic_tpye = 7;
//    judge_txmsg.ext_client_graphic_draw.start_angle = 25;
//    judge_txmsg.ext_client_graphic_draw.end_angle = 6;
//    judge_txmsg.ext_client_graphic_draw.layer = 9;
//    judge_txmsg.ext_client_graphic_draw.color = 0;
//    judge_txmsg.ext_client_graphic_draw.width = 3;
//    judge_txmsg.ext_client_graphic_draw.start_x = 200;
//    judge_txmsg.ext_client_graphic_draw.start_y = 800;
//    judge_txmsg.ext_client_graphic_draw.radius = 0;
//    judge_txmsg.ext_client_graphic_draw.end_x = 0;
//    judge_txmsg.ext_client_graphic_draw.end_y = 0;
//    for (i = 0; i < 30; i++)
//        judge_txmsg.text[i] = 0;
//    memcpy(judge_txmsg.text, Str, sizeof(Str) + 3);
//    sprintf((char *)&StrTmp, "%4.2f", tmp);
//    memcpy(judge_txmsg.text + sizeof(Str) + 3, StrTmp, sizeof(StrTmp));

//    /*数据封装*/
//    data_packet_pack(0x0301, (uint8_t *)&judge_txmsg, sizeof(judge_txmsg), 0xA5);
//    /*DMA发送，清除字符串*/
//    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
//    memset(&StrTmp, 0, sizeof(StrTmp));
//}

/**
  * @brief  发送Visionz数据
  * @param  uint16_t sender_ID发送端ID,uint8_t *Str,float tmp,uint8_t operate_tpye对图像的操作
  * @retval void
  * @attention 
  */
void JudgeUI_SendVisionzData(uint16_t sender_ID, uint8_t *Str, float tmp, uint8_t operate_tpye)
{
    uint8_t i;
    judge_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0110;
    judge_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
    judge_txmsg.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    judge_txmsg.ext_client_graphic_draw.graphic_name[0] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[1] = '0';
    judge_txmsg.ext_client_graphic_draw.graphic_name[2] = '4';
    judge_txmsg.ext_client_graphic_draw.operate_tpye = operate_tpye;
    judge_txmsg.ext_client_graphic_draw.graphic_tpye = 7;
    judge_txmsg.ext_client_graphic_draw.start_angle = 25;
    judge_txmsg.ext_client_graphic_draw.end_angle = 6;
    judge_txmsg.ext_client_graphic_draw.layer = 9;
    judge_txmsg.ext_client_graphic_draw.color = 0;
    judge_txmsg.ext_client_graphic_draw.width = 3;
    judge_txmsg.ext_client_graphic_draw.start_x = 200;
    judge_txmsg.ext_client_graphic_draw.start_y = 800;
    judge_txmsg.ext_client_graphic_draw.radius = 0;
    judge_txmsg.ext_client_graphic_draw.end_x = 0;
    judge_txmsg.ext_client_graphic_draw.end_y = 0;
    for (i = 0; i < 30; i++)
        judge_txmsg.text[i] = 0;
    memcpy(judge_txmsg.text, Str, sizeof(Str) + 3);
    sprintf((char *)&StrTmp, "%4.2f", tmp);
    memcpy(judge_txmsg.text + sizeof(Str) + 3, StrTmp, sizeof(StrTmp));

    /*数据封装*/
    data_packet_pack(0x0301, (uint8_t *)&judge_txmsg, sizeof(judge_txmsg), 0xA5);
    /*DMA发送，清除字符串*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
    memset(&StrTmp, 0, sizeof(StrTmp));
}


/**
  * @brief  发送底盘位置线条图形、线条框以及加粗瞄准线条（不会变化）
  * @param  uint16_t sender_ID发送端ID, uint16_t len线条长度, uint8_t operate_tpye对图像的操作
  * @retval void
  * @attention 
  */
void JudgeUI_SendBKG2_SuperCap_Line(uint16_t sender_ID, uint16_t len, uint8_t operate_tpye)
{
    ext_client_graphic_drawseven.ext_client_graphic_basic.data_comment_ID = 0x0104;
    ext_client_graphic_drawseven.ext_client_graphic_basic.sender_ID = sender_ID;
    ext_client_graphic_drawseven.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);

    ext_client_graphic_drawseven.ext_client_graphic_draw[0].operate_tpye = operate_tpye;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].operate_tpye = operate_tpye;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].operate_tpye = operate_tpye;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].operate_tpye = operate_tpye;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].operate_tpye = operate_tpye; //变化线条
	ext_client_graphic_drawseven.ext_client_graphic_draw[5].operate_tpye = operate_tpye;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].operate_tpye = operate_tpye;
	
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[2] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].layer = 9;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].color = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].width = 2;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_x = 500;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_y = 100;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_x = 760;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_y = 200;

    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[2] = '1';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].color = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].width = 2;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_x = 760;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_y = 200;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_x = 920;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_y = 200;

    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[2] = '2';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].color = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].width = 2;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_x = 1000;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_y = 200;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_x = 1160;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_y = 200;

    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[2] = '3';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].color = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].width = 2;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_x = 1160;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_y = 200;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_x = 1420;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_y = 100;

    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[2] = '4';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].layer = 9;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].color = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].width = 5;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_x = 1610;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_y = 800;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_x = 1610+len;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_y = 800;
	
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[2] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].width = 4;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_x = 920;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_y = 550;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_x = 1000;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_y = 550;

    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_tpye = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[0] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[2] = '6';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].color = 5;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].width = 2;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_x = 1607;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_y = 803;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_x = 1912;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_y = 794;

    /*数据封装*/
    data_packet_pack((0x0301), (uint8_t *)&ext_client_graphic_drawseven, sizeof(ext_client_graphic_drawseven_t), 0xA5);

    /*DMA发送*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}

/**
  * @brief  发送瞄准目标图形（不会变化）
  * @param  uint16_t sender_ID发送端ID, uint16_t client_ID接收端ID暂时不用管, uint8_t operate_tpye对图像的操作
  * @retval void
  * @attention 
  */
void JudgeUI_SendBKG1(uint16_t sender_ID,uint8_t operate_tpye)
{

    int8_t i;
    ext_client_graphic_drawseven.ext_client_graphic_basic.data_comment_ID = 0x0104;
    ext_client_graphic_drawseven.ext_client_graphic_basic.sender_ID = sender_ID;
    ext_client_graphic_drawseven.ext_client_graphic_basic.client_ID = get_client_ID(sender_ID);
    for (i = 0; i < 7; i++)
    {
        ext_client_graphic_drawseven.ext_client_graphic_draw[i].operate_tpye = operate_tpye;
    }

    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].graphic_name[2] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_x = 950;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_y = 270;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_x = 970;
    ext_client_graphic_drawseven.ext_client_graphic_draw[0].end_y = 270;

    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].graphic_name[2] = '1';
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_x = 930;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_y = 300;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_x = 990;
    ext_client_graphic_drawseven.ext_client_graphic_draw[1].end_y = 300;

    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].graphic_name[2] = '2';
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_x = 900;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_y = 350;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_x = 1020;
    ext_client_graphic_drawseven.ext_client_graphic_draw[2].end_y = 350;

    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].graphic_name[2] = '3';
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_x = 880;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_y = 400;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_x = 1040;
    ext_client_graphic_drawseven.ext_client_graphic_draw[3].end_y = 400;

    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].graphic_name[2] = '4';
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_x = 850;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_y = 450;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_x = 1070;
    ext_client_graphic_drawseven.ext_client_graphic_draw[4].end_y = 450;

    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_tpye = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].graphic_name[2] = '5';
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_x = 960;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_y = 200;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].radius = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_x = 960;
    ext_client_graphic_drawseven.ext_client_graphic_draw[5].end_y = 600;

    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_tpye = 2;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[0] = '7';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[1] = '0';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].graphic_name[2] = '6';
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].layer = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].color = 6;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].width = 1;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_x = 960;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_y = 400;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].start_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_angle = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].radius = 150;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_x = 0;
    ext_client_graphic_drawseven.ext_client_graphic_draw[6].end_y = 0;

    /*数据封装*/
    data_packet_pack((0x0301), (uint8_t *)&ext_client_graphic_drawseven, sizeof(ext_client_graphic_drawseven_t), 0xA5);

    /*DMA发送*/
    HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}



///**
//* @brief  俯仰角发送
//  * @param  void
//  * @retval void
//  * @attention 
//  */
//void JudgeUI_PitchData(uint8_t robot_id)
//{
//	static uint8_t tick = 0;
//	uint8_t Strp[] = "Pitch: ";
//	if(tick < 5)
//	{
//		JudgeUI_SendHolderPitchData((uint16_t)(robot_id&0x00FF),Strp, (float)Holder.Pitch.angle,2);
//	}
//	else
//	{
//		JudgeUI_SendHolderPitchData((uint16_t)(robot_id&0x00FF),Strp, (float)Holder.Pitch.angle,1);
//		if(tick == 5)
//			tick = 0;
//	}
//	tick++;
//}

/**
* @brief  视觉数据z发送
  * @param  void
  * @retval void
  * @attention 
  */
//void JudgeUI_VisionzData(uint8_t robot_id)
//{
//	static uint8_t tick = 0;
//	uint8_t Strp[] = "VData_z: ";
//	if(tick < 5)
//	{
//		JudgeUI_SendVisionzData((uint16_t)(robot_id&0x00FF),Strp, (float)vData_z,2);
//	}
//	else
//	{
//		JudgeUI_SendVisionzData((uint16_t)(robot_id&0x00FF),Strp, (float)vData_z,1);
//		if(tick == 5)
//			tick = 0;
//	}
//	tick++;
//}

/**
* @brief  小陀螺状态发送 
  * @param  uint8_t state 'f':follow  'l':spin
  * @retval void
  * @attention 
  */
void JudgeUI_SpinMode(uint8_t robot_id, uint8_t state)
{
	static uint8_t tick = 0;
	if(tick < 5)
	{
		JudgeUI_SendSpinMode((uint16_t)(robot_id&0x00FF),state,2);
	}
	else
	{
		JudgeUI_SendSpinMode((uint16_t)(robot_id&0x00FF),state,1);
		if(tick == 5)
			tick = 0;
	}
	tick++;
}

/**
* @brief  打符状态发送 
  * @param  uint8_t state 'W':WINDMILL  'A':AUTO
  * @retval void
  * @attention 
  */
void JudgeUI_Vision_State(uint8_t robot_id, uint8_t state)
{
	static uint8_t tick = 0;
	if(tick < 5)
	{
		JudgeUI_SendVisionState((uint16_t)(robot_id&0x00FF),state,2);
	}
	else
	{
		JudgeUI_SendVisionState((uint16_t)(robot_id&0x00FF),state,1);
		if(tick == 5)
			tick = 0;
	}
	tick++;
}

/**
  * @brief  弹舱盖状态发送
  * @param  uint8_t state Y/N
  * @retval void
  * @attention 
  */
void JudgeUI_LidState(uint8_t robot_id, uint8_t Lid_St)
{
	static uint8_t tick = 0;
    if(tick < 5)
	{
		JudgeUI_SendLidSt((uint16_t)(robot_id&0x00FF),Lid_St,2);
	}
	else
	{
		JudgeUI_SendLidSt((uint16_t)(robot_id&0x00FF),Lid_St,1);
		if(tick == 5)
			tick = 0;
	}
	tick++;

}

/**
  * @brief  发送哨兵自主决策控制代码
  * @param  uint16_t sender_ID发送端ID
  * @retval void
  * @attention 
  */
HAL_StatusTypeDef DMA_value;
void JudgeUI_SentryControl(uint16_t sender_ID,uint32_t sentry_cmd)
{
    sentry_control_txmsg.ext_client_graphic_basic.data_comment_ID = 0x0120;
    sentry_control_txmsg.ext_client_graphic_basic.sender_ID = sender_ID;
    sentry_control_txmsg.ext_client_graphic_basic.client_ID = 0x8080;
    
		sentry_control_txmsg.sentry_cmd = sentry_cmd;

    /*数据封装*/
    data_packet_pack((0x0301), (uint8_t *)&sentry_control_txmsg, sizeof(sentry_control_txmsg), 0xA5);

    /*DMA发送*/
    DMA_value = HAL_UART_Transmit_DMA(&huart3, uart3_tx_buf, sizeof(uart3_tx_buf));
}
/************************ (C) COPYRIGHT CSU_RM_FYT *************END OF FILE****/
