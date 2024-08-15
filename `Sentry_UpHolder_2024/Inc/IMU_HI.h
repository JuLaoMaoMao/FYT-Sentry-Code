/**
  ******************************************************************************
  * @file    IMU_HI.h
  * @author  Brosy
  * @brief   HI219������
  * @date    2022-11-17
  ******************************************************************************
  * @attention HI219����ͨ��Э��
  *
  * Copyright (c) CSU_RM_FYT.
  * All rights reserved.
  *
  * This software component is licensed by Brosy under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  * opensource.org/licenses/BSD-3-Clause
  ******************************************************************************
  *�޸ļ�¼��
  *<ʱ��>      |<�汾>      |<����>      |<����>    
  *2022-11-17  |v1.0        |Brosy       |ֱ����ֲ�ٷ�����δ���޸�
  ******************************************************************************
**/
/* �ļ���ǰ���������»��ߡ�__��������� ��_H�� */
#ifndef __IMU_HI_H
#define __IMU_HI_H

/* includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* define --------------------------------------------------------------------*/
#define MAX_PACKET_LEN          (128)

/* typedef -------------------------------------------------------------------*/
typedef enum 
{
    kItemKeyStatus =            0x80,   /* key status           size: 4 */
    kItemID =                   0x90,   /* user programed ID    size: 1 */
    kItemUID =                  0x91,   /* Unique ID            size: 4 */
    kItemIPAdress =             0x92,   /* ip address           size: 4 */
    kItemAccRaw =               0xA0,   /* raw acc              size: 3x2 */
    kItemAccCalibrated =        0xA1,
    kItemAccFiltered =          0xA2,   
    kItemAccLinear =            0xA5,
    kItemGyoRaw =               0xB0,   /* raw gyro             size: 3x2 */  
    kItemGyoCalibrated =        0xB1,
    kItemGyoFiltered =          0xB2, 
    kItemMagRaw =               0xC0,   /* raw mag              size: 3x2 */
    kItemMagCalibrated =        0xC1,
    kItemMagFiltered =          0xC2,
    kItemRotationEular =        0xD0,   /* eular angle          size:3x2 */
    kItemRotationEular2 =       0xD9,   /* new eular angle      size:3x4 */
    kItemRotationQuat =         0xD1,   /* att q,               size:4x4 */
    kItemTemperature =          0xE0,   
    kItemPressure =             0xF0,   /* pressure             size:1x4 */
    kItemEnd =                  0x00,   
}ItemID_t;

typedef struct
{
    uint32_t ofs;
    uint8_t buf[MAX_PACKET_LEN];    /* total frame buffer */
    uint16_t payload_len;           
    uint16_t len;                   /* total frame len */
    uint8_t type;
}Packet_t;

/* variables -----------------------------------------------------------------*/

/* function ------------------------------------------------------------------*/
/* packet Tx API */
uint32_t Packet_AddData(Packet_t *pkt, uint8_t *buf, uint16_t len);
uint32_t Packet_Begin(Packet_t *pkt);
uint32_t Packet_Final(Packet_t *pkt);
uint32_t Packet_CreatePing(Packet_t *pkt);
uint32_t Packet_CreatePingAck(Packet_t *pkt, uint8_t major, uint8_t minor, uint8_t bugfix, uint16_t option);

/* packet Rx API */
typedef void (*OnDataReceivedEvent)(Packet_t *pkt);
void Packet_DecodeInit(Packet_t *pkt, OnDataReceivedEvent rx_handler);
void Packet_DecodeInit_Chassis(Packet_t *pkt, OnDataReceivedEvent rx_handler);
uint32_t Packet_Decode(uint8_t c);
uint32_t Packet_Decode_Chassis(uint8_t c);


int imu_data_decode_init(void);
int imu_data_decode_init_chassis(void);
int get_raw_acc(int16_t* a);
int get_raw_gyo(int16_t* g);
int get_raw_gyo_chassis(int16_t* g);
int get_raw_mag(int16_t* m);
int get_id(uint8_t *user_id);
int get_eular(float* e);
int get_eular_chassis(float* e);
int get_quat(float* q);
#endif

