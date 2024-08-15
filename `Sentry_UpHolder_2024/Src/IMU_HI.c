/**
  ******************************************************************************
  * @file    IMU_HI.c
  * @author  Brosy
  * @brief   HI219陀螺仪
  * @date    2022-11-17
  ******************************************************************************
  * @attention HI219串口通信协议
  *
  * Copyright (c) CSU_RM_FYT.
  * All rights reserved.
  *
  * This software component is licensed by Brosy under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  ******************************************************************************
  *修改记录：
  *<时间>      |<版本>      |<作者>      |<描述>    
  *2022-03-29  |v1.0        |Brosy       |直接移植官方程序，未做修改
  ******************************************************************************
**/
/* includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "IMU_HI.h"

/* define --------------------------------------------------------------------*/
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))
#endif

#ifndef CH_OK
#define CH_OK   (0)
#endif

#ifndef CH_ERR
#define CH_ERR  (1)
#endif

/* typedef -------------------------------------------------------------------*/
enum status
{
    kStatus_Idle,
    kStatus_Cmd,
    kStatus_LenLow,
    kStatus_LenHigh,
    kStatus_CRCLow,
    kStatus_CRCHigh,
    kStatus_Data,
};

/* variables -----------------------------------------------------------------*/

/* function ------------------------------------------------------------------*/
static void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
{
    uint32_t crc = *currectCrc;
    uint32_t j;
    for (j=0; j < lengthInBytes; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    *currectCrc = crc;
}

uint32_t Packet_CreatePing(Packet_t *pkt)
{
    pkt->buf[0] = 0x5A;
    pkt->buf[1] = 0xA6;
    pkt->payload_len = 0;
    pkt->len = 2;
    return CH_OK;
}

uint32_t Packet_CreatePingAck(Packet_t *pkt, uint8_t major, uint8_t minor, uint8_t bugfix, uint16_t option)
{
    pkt->buf[0] = 0x5A;
    pkt->buf[1] = 0xA7;

    /* protocol bug fix */
    pkt->buf[2] = bugfix;
    /* protocol minor */
    pkt->buf[3] = minor;
    /* protocol major */
    pkt->buf[4] = major;
    pkt->buf[5] = 'P';

    /* option low: sender's address low */
    pkt->buf[6] = (option & 0x00FF)>>0;

    /* option high: sender's address high */
    pkt->buf[7] = (option & 0xFF00)>>8;

    /* crc */
    uint16_t crc;
    crc = 0;
    crc16_update(&crc, &pkt->buf[0], 8);
    pkt->buf[8] = (crc & 0x00FF)>>0;
    pkt->buf[9] = (crc & 0xFF00)>>8;

    pkt->payload_len = 0;
    pkt->type = 0xA7;
    pkt->len = 10;
    return CH_OK;
}

uint32_t Packet_Begin(Packet_t *pkt)
{
    pkt->ofs = 6; /* sof(2) len(2) + crc(2) */
    memset(&pkt->buf[0], 0, sizeof(pkt->buf));
    pkt->buf[0] = 0x5A; /* header */
    pkt->buf[1] = 0xA5; /* data packet */
    return CH_OK;
}

uint32_t Packet_AddData(Packet_t *pkt, uint8_t *buf, uint16_t len)
{
    /* add item content into buffer */
    memcpy((pkt->buf + pkt->ofs), buf, len);
    pkt->ofs += len;
    return CH_OK;
}

uint32_t Packet_Final(Packet_t *pkt)
{

    pkt->payload_len = pkt->ofs -6;
    pkt->len = pkt->ofs;

    pkt->buf[2] = (pkt->payload_len & 0x00FF)>>0;
    pkt->buf[3] = (pkt->payload_len & 0xFF00)>>8;

    /* crc */
    uint16_t crc;
    crc = 0;
    crc16_update(&crc, &pkt->buf[0], 4);
    crc16_update(&crc, &pkt->buf[6], pkt->payload_len);
    pkt->buf[4] = (crc & 0x00FF)>>0;
    pkt->buf[5] = (crc & 0xFF00)>>8;

    return CH_OK;
}

/* function pointer */
static OnDataReceivedEvent EventHandler;
static OnDataReceivedEvent EventHandler_Chassis;
static Packet_t *RxPkt;
static Packet_t *RxPkt_Chassis;

/**
* @brief  云台陀螺仪初始化姿态解码模块
* @note   完成初始化一个引脚配置
* @param  pkt 接收包指针
* @param  接收成功回调函数
* @code

*      void OnDataReceived(Packet_t *pkt)
*      {
*          pkt->buf 为数据 pkt->payload_len 为接收到的字节长度
*      }
*
*      Packet_t pkt;
*      Packet_DecodeInit(&pkt, OnDataReceived);
* @endcode
* @retval None
*/
void Packet_DecodeInit(Packet_t *pkt, OnDataReceivedEvent Func)
{
    EventHandler = Func;
    memset(pkt, 0, sizeof(Packet_t));
    RxPkt = pkt;
}

/**
* @brief  底盘陀螺仪初始化姿态解码模块
* @note   完成初始化一个引脚配置
* @param  pkt 接收包指针
* @param  接收成功回调函数
* @code

*      void OnDataReceived(Packet_t *pkt)
*      {
*          pkt->buf 为数据 pkt->payload_len 为接收到的字节长度
*      }
*
*      Packet_t pkt;
*      Packet_DecodeInit(&pkt, OnDataReceived);
* @endcode
* @retval None
*/
void Packet_DecodeInit_Chassis(Packet_t *pkt, OnDataReceivedEvent Func)
{
    EventHandler_Chassis = Func;
    memset(pkt, 0, sizeof(Packet_t));
    RxPkt_Chassis = pkt;
}

/**
* @brief  接收云台IMU数据
* @note   在串口接收中断中调用此函数
* @param  c 串口数据
* @retval CH_OK
*/
uint32_t Packet_Decode(uint8_t c)
{
    static uint16_t CRCReceived = 0;            /* CRC value received from a frame */
    static uint16_t CRCCalculated = 0;          /* CRC value caluated from a frame */
    static uint8_t status = kStatus_Idle;       /* state machine */
    static uint8_t crc_header[4] = {0x5A, 0xA5, 0x00, 0x00};

    switch(status)
    {
    case kStatus_Idle:
        if(c == 0x5A)
            status = kStatus_Cmd;
        break;
    case kStatus_Cmd:
        RxPkt->type = c;
        switch(RxPkt->type)
        {
        case 0xA5:  /* Data */
            status = kStatus_LenLow;
            break;
        case 0xA6:  /* Ping */
            if(EventHandler != NULL)
            {
                EventHandler(RxPkt);
            }
            status = kStatus_Idle;
            break;
        case 0xA7:  /* Ping Respond */
            RxPkt->ofs = 0;
            status = kStatus_Data;
            break;
        }
        break;
    case kStatus_LenLow:
        RxPkt->payload_len = c;
        crc_header[2] = c;
        status = kStatus_LenHigh;
        break;
    case kStatus_LenHigh:
        RxPkt->payload_len |= (c<<8);
        crc_header[3] = c;
        status = kStatus_CRCLow;
        break;
    case kStatus_CRCLow:
        CRCReceived = c;
        status = kStatus_CRCHigh;
        break;
    case kStatus_CRCHigh:
        CRCReceived |= (c<<8);
        RxPkt->ofs = 0;
        CRCCalculated = 0;
        status = kStatus_Data;
        break;
    case kStatus_Data:
        RxPkt->buf[RxPkt->ofs++] = c;
        if(RxPkt->type == 0xA7 && RxPkt->ofs >= 8)
        {
            RxPkt->payload_len = 8;
            EventHandler(RxPkt);
            status = kStatus_Idle;
        }

        if(RxPkt->ofs >= RxPkt->payload_len && RxPkt->type == 0xA5)
        {
            /* calculate CRC */
            crc16_update(&CRCCalculated, crc_header, 4);
            crc16_update(&CRCCalculated, RxPkt->buf, RxPkt->ofs);

            /* CRC match */
            if(CRCCalculated == CRCReceived)
            {
                EventHandler(RxPkt);
            }
            status = kStatus_Idle;
        }
        break;
    default:
        status = kStatus_Idle;
        break;
    }
    return CH_OK;
}

/**
* @brief  接收云台IMU数据
* @note   在串口接收中断中调用此函数
* @param  c 串口数据
* @retval CH_OK
*/
uint32_t Packet_Decode_Chassis(uint8_t c)
{
    static uint16_t CRCReceived = 0;            /* CRC value received from a frame */
    static uint16_t CRCCalculated = 0;          /* CRC value caluated from a frame */
    static uint8_t status = kStatus_Idle;       /* state machine */
    static uint8_t crc_header[4] = {0x5A, 0xA5, 0x00, 0x00};

    switch(status)
    {
    case kStatus_Idle:
        if(c == 0x5A)
            status = kStatus_Cmd;
        break;
    case kStatus_Cmd:
        RxPkt_Chassis->type = c;
        switch(RxPkt_Chassis->type)
        {
        case 0xA5:  /* Data */
            status = kStatus_LenLow;
            break;
        case 0xA6:  /* Ping */
            if(EventHandler_Chassis != NULL)
            {
                EventHandler_Chassis(RxPkt_Chassis);
            }
            status = kStatus_Idle;
            break;
        case 0xA7:  /* Ping Respond */
            RxPkt_Chassis->ofs = 0;
            status = kStatus_Data;
            break;
        }
        break;
    case kStatus_LenLow:
        RxPkt_Chassis->payload_len = c;
        crc_header[2] = c;
        status = kStatus_LenHigh;
        break;
    case kStatus_LenHigh:
        RxPkt_Chassis->payload_len |= (c<<8);
        crc_header[3] = c;
        status = kStatus_CRCLow;
        break;
    case kStatus_CRCLow:
        CRCReceived = c;
        status = kStatus_CRCHigh;
        break;
    case kStatus_CRCHigh:
        CRCReceived |= (c<<8);
        RxPkt_Chassis->ofs = 0;
        CRCCalculated = 0;
        status = kStatus_Data;
        break;
    case kStatus_Data:
        RxPkt_Chassis->buf[RxPkt_Chassis->ofs++] = c;
        if(RxPkt_Chassis->type == 0xA7 && RxPkt_Chassis->ofs >= 8)
        {
            RxPkt_Chassis->payload_len = 8;
            EventHandler_Chassis(RxPkt_Chassis);
            status = kStatus_Idle;
        }

        if(RxPkt_Chassis->ofs >= RxPkt_Chassis->payload_len && RxPkt_Chassis->type == 0xA5)
        {
            /* calculate CRC */
            crc16_update(&CRCCalculated, crc_header, 4);
            crc16_update(&CRCCalculated, RxPkt_Chassis->buf, RxPkt_Chassis->ofs);

            /* CRC match */
            if(CRCCalculated == CRCReceived)
            {
                EventHandler_Chassis(RxPkt_Chassis);
            }
            status = kStatus_Idle;
        }
        break;
    default:
        status = kStatus_Idle;
        break;
    }
    return CH_OK;
}

static int16_t acc[3];
static int16_t gyo[3];
static int16_t mag[3];
static float eular[3];
static float quat[4];
static uint8_t id;
//云台陀螺仪

static int16_t gyo_chassis[3];
static float eular_chassis[3];
//底盘陀螺仪

int get_raw_acc(int16_t* a)
{
    memcpy(a, acc, sizeof(acc));
    return 0;
}

int get_raw_gyo(int16_t* g)
{
    memcpy(g, gyo, sizeof(gyo));
    return 0;
}

int get_raw_gyo_chassis(int16_t* g)
{
    memcpy(g, gyo_chassis, sizeof(gyo_chassis));
    return 0;
}

int get_raw_mag(int16_t* m)
{
    memcpy(m, mag, sizeof(mag));
    return 0;
}

int get_eular(float* e)
{
    memcpy(e, eular, sizeof(eular));
    return 0;
}

int get_eular_chassis(float* e)
{
    memcpy(e, eular_chassis, sizeof(eular_chassis));
    return 0;
}

int get_quat(float* q)
{
    memcpy(q, quat, sizeof(quat));
    return 0;
}

int get_id(uint8_t *user_id)
{
    *user_id = id;
    return 0;
}

/*  callback function of  when recv a data frame successfully */
static void OnDataReceived(Packet_t *pkt)
{
    int offset = 0;
    uint8_t *p = pkt->buf;
    while(offset < pkt->payload_len)
    {
        switch(p[offset])
        {
        case kItemID:
            id = p[1];
            offset += 2;
            break;
        case kItemAccRaw:
        case kItemAccCalibrated:
        case kItemAccFiltered:
        case kItemAccLinear:
            memcpy(acc, p + offset + 1, sizeof(acc));
            offset += 7;
            break;
        case kItemGyoRaw:
        case kItemGyoCalibrated:
        case kItemGyoFiltered:
            memcpy(gyo, p + offset + 1, sizeof(gyo));
            offset += 7;
            break;
        case kItemMagRaw:
        case kItemMagCalibrated:
        case kItemMagFiltered:
            memcpy(mag, p + offset + 1, sizeof(mag));
            offset += 7;
            break;
        case kItemRotationEular:
            eular[0] = ((float)(int16_t)(p[offset+1] + (p[offset+2]<<8)))/100;
            eular[1] = ((float)(int16_t)(p[offset+3] + (p[offset+4]<<8)))/100;
            eular[2] = ((float)(int16_t)(p[offset+5] + (p[offset+6]<<8)))/10;
            offset += 7;
            break;
        case kItemRotationEular2:
            memcpy(eular, p + offset + 1, sizeof(eular));
            offset += 13;
            break;
        case kItemRotationQuat:
            memcpy(quat, p + offset + 1, sizeof(quat));
            offset += 17;
            break;
        case kItemPressure:
            offset += 5;
            break;
        case kItemTemperature:
            offset += 5;
            break;
        default:
            offset++;
            //printf("data decode wrong\r\n");
            break;
        }
    }

}

static void OnDataReceived_Chassis(Packet_t *pkt)
{
    int offset = 0;
    uint8_t *p = pkt->buf;
    while(offset < pkt->payload_len)
    {
        switch(p[offset])
        {
        case kItemID:
            id = p[1];
            offset += 2;
            break;
        case kItemAccRaw:
        case kItemAccCalibrated:
        case kItemAccFiltered:
        case kItemAccLinear:
            memcpy(acc, p + offset + 1, sizeof(acc));
            offset += 7;
            break;
        case kItemGyoRaw:
        case kItemGyoCalibrated:
        case kItemGyoFiltered:
            memcpy(gyo_chassis, p + offset + 1, sizeof(gyo_chassis));
            offset += 7;
            break;
        case kItemMagRaw:
        case kItemMagCalibrated:
        case kItemMagFiltered:
            memcpy(mag, p + offset + 1, sizeof(mag));
            offset += 7;
            break;
        case kItemRotationEular:
            eular_chassis[0] = ((float)(int16_t)(p[offset+1] + (p[offset+2]<<8)))/100;
            eular_chassis[1] = ((float)(int16_t)(p[offset+3] + (p[offset+4]<<8)))/100;
            eular_chassis[2] = ((float)(int16_t)(p[offset+5] + (p[offset+6]<<8)))/10;
            offset += 7;
            break;
        case kItemRotationEular2:
            memcpy(eular_chassis, p + offset + 1, sizeof(eular_chassis));
            offset += 13;
            break;
        case kItemRotationQuat:
            memcpy(quat, p + offset + 1, sizeof(quat));
            offset += 17;
            break;
        case kItemPressure:
            offset += 5;
            break;
        case kItemTemperature:
            offset += 5;
            break;
        default:
            offset++;
            //printf("data decode wrong\r\n");
            break;
        }
    }

}

static Packet_t rx_pkt_;
static Packet_t rx_pkt_chassis;
int imu_data_decode_init(void)
{
    Packet_DecodeInit(&rx_pkt_, OnDataReceived);
    return 0;
}

int imu_data_decode_init_chassis(void)
{
		Packet_DecodeInit_Chassis(&rx_pkt_chassis, OnDataReceived_Chassis);
    return 0;
}
