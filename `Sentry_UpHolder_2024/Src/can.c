/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "message.h"
#include "chassis.h"
#include "config.h"
#include "Holder.h"
#include "shoot.h"
#include "tim.h"
#include "usart.h"
#include <stdlib.h>
 
#define F407_CAN_ID 0x200
#define CAN1_FIFO CAN_RX_FIFO0
#define CAN2_FIFO CAN_RX_FIFO0

//标定偏差角，把angle_bias = 0;读取dir_angle的值，angle_bias取该值的相反数，从而使得dir_angle = 0;
float dir_angle_shoot = 0;
float dir_angle_camera = 0;
float bias_angle_shoot = -3070;//-1000
float bias_angle_camera = 0;
//陀螺仪参数
float dev_imu_pitch = 1800; /* 陀螺仪安装偏移角度°：俯仰 */

HAL_StatusTypeDef can1_value;
HAL_StatusTypeDef can2_value;
float can_state;
float Yaw,Pitch;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_TX_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN1_FilterInit(void)
{
    CAN_FilterTypeDef fcan;
    fcan.FilterBank = 0;
    fcan.FilterMode = CAN_FILTERMODE_IDMASK;
    fcan.FilterScale = CAN_FILTERSCALE_32BIT;
    
    fcan.FilterIdHigh = 0;
    fcan.FilterIdLow = 0;
    fcan.FilterMaskIdHigh = 0;
    fcan.FilterMaskIdLow = 0;
    fcan.FilterFIFOAssignment = CAN1_FIFO;
    fcan.FilterActivation = ENABLE;
    fcan.SlaveStartFilterBank = 0;
    
    HAL_CAN_ConfigFilter(&hcan1,&fcan);
    HAL_CAN_Start(&hcan1);
    
	  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
}

void CAN2_FilterInit(void)
{
    CAN_FilterTypeDef fcan;
    fcan.FilterBank = 15;
    fcan.FilterMode = CAN_FILTERMODE_IDMASK;
    fcan.FilterScale = CAN_FILTERSCALE_32BIT;
    
    fcan.FilterIdHigh = 0;
    fcan.FilterIdLow = 0;
    fcan.FilterMaskIdHigh = 0;
    fcan.FilterMaskIdLow = 0;
    fcan.FilterFIFOAssignment = CAN2_FIFO;
    fcan.FilterActivation = ENABLE;
    fcan.SlaveStartFilterBank = 15;
    
    HAL_CAN_ConfigFilter(&hcan2,&fcan);
    HAL_CAN_Start(&hcan2);
    
    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY );
}

/**
  * @brief  CAN1、CAN2发送数据
  * @param  ID、数据
  * @retval void
  * @attention
  */
CAN_TxHeaderTypeDef CAN1TxMsg;
void CAN1_Transmit(uint16_t ID,uint8_t *pData)
{
		uint32_t MailBox;
    CAN1TxMsg.StdId = ID;
    CAN1TxMsg.ExtId = 0;
    CAN1TxMsg.IDE = CAN_ID_STD;
    CAN1TxMsg.RTR = CAN_RTR_DATA;
    CAN1TxMsg.DLC = 8;
//		while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
		can1_value = HAL_CAN_AddTxMessage(&hcan1,&CAN1TxMsg,pData,&MailBox);
		
		if(flag)
		{
			if(can1_value == HAL_OK)	
				can_state = 100000;//用于检查can1/2是否成功发送
			
			if(can1_value == HAL_ERROR) 
				 can_state = 0;
		}
		else 
			can_state = 0;
}

CAN_TxHeaderTypeDef CAN2TxMsg;
void CAN2_Transmit(uint16_t ID,uint8_t *pData)
{
	  uint32_t MailBox;
    CAN2TxMsg.StdId = ID;
    CAN2TxMsg.ExtId = 0;
    CAN2TxMsg.IDE = CAN_ID_STD;
    CAN2TxMsg.RTR = CAN_RTR_DATA;
    CAN2TxMsg.DLC = 8;
		while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0);
		can2_value = HAL_CAN_AddTxMessage(&hcan2,&CAN2TxMsg,pData,&MailBox); 
		if(flag)
		{
			if(can2_value == HAL_OK)	
				can_state = 100000;
		
			if(can2_value == HAL_ERROR) 
				can_state = 0;
		}
		else 
			can_state = 0;
}

/**
  * @brief  CAN接收回调
  * @param  CAN结构
  * @retval void
  * @attention
  */
int16_t watch_sup = 0;
int16_t watch_speed = 0;
int16_t watch_current = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
    UNUSED(hcan);
    uint8_t Data[8];
    CAN_RxHeaderTypeDef RxMsg;
	
    if(hcan->Instance == CAN1)
    {
        HAL_CAN_GetRxMessage(hcan, CAN1_FIFO, &RxMsg, Data);
        switch(RxMsg.StdId)
        {						  					
						case 0x201:  /* M3508左摩擦轮 */
									Observer.Rx.CAN2_0x201_Rate++;
									Shoot_M3508[0].Speed = Data[2] << 8 | Data[3];
		//				      Chassis_CanTransmit();
						break;
			
						case 0x202:/* M3508右摩擦轮 */
									Observer.Rx.CAN2_0x202_Rate++;
									Shoot_M3508[1].Speed = Data[2] << 8 | Data[3];
		//							Chassis_CanTransmit();
						break;
						
						case 0x203:  /* M2006拨弹电机 */
							Observer.Rx.CAN1_0x204_Rate++;
							CircleContinue(&Shoot_M2006.Mcircle_Position, Data[0] << 8 | Data[1],4096);
							Shoot_M2006.Angle = Shoot_M2006.Mcircle_Position.Circle + Shoot_M2006.Mcircle_Position.Angle/8192;
							Shoot_M2006.Speed = Data[2] << 8 | Data[3];	
//              Shoot_CanTransmit();						
						break;
						
						case 0x205:  /* GM6020云台偏航：Yaw  */
							Observer.Rx.CAN1_0x20A_Rate++;
							CircleContinue(&Holder.Yaw_0x20A.Position, Data[0] << 8 | Data[1],4096);
              Holder.Yaw_0x20A.Angle = 8192 * Holder.Yaw_0x20A.Position.Circle + Holder.Yaw_0x20A.Position.Angle;
							Holder.Yaw_0x20A.Speed = Data[2] << 8 | Data[3];
//						  Holder_CanTransmit();
						break;
						
						case 0x206:  /* GM6020云台俯仰：Pitch  */
							Observer.Rx.CAN1_0x20B_Rate++;
							CircleContinue(&Holder.Pitch_0x20B.Position, Data[0] << 8 | Data[1],4096);
              Holder.Pitch_0x20B.Angle = (8192 * Holder.Pitch_0x20B.Position.Circle + Holder.Pitch_0x20B.Position.Angle);
							Holder.Pitch_0x20B.Motor_Angle = (8192 * Holder.Pitch_0x20B.Position.Circle + Holder.Pitch_0x20B.Position.Angle) * 100;
							Holder.Pitch_0x20B.Speed = Data[2] << 8 | Data[3];
							Holder.Pitch_0x20B.Motor_Speed = Holder.Pitch_0x20B.Speed;
						  Holder.Pitch_0x20B.Current = Data[4] << 8 | Data[5];
//						  Holder_CanTransmit();
						break;

				}
     } 
		
		if(hcan->Instance == CAN2)
		{
	   	HAL_CAN_GetRxMessage(hcan, CAN2_FIFO, &RxMsg, Data);
		  switch(RxMsg.StdId)
			{		
				case 0x203:
							Observer.Rx.CAN2_0x201_Rate++;
							Remote_Data.S1 = Data[0];
							Remote_Data.S2 = Data[1];
							Yaw = (int16_t)Data[2]<<8 |(int16_t)Data[3];
							Pitch = (int16_t)Data[4]<<8 | (int16_t)Data[5];
							if(Yaw >665)Yaw = Yaw - 65536;
							if(Pitch >665)Pitch = Pitch - 65536;
							Remote_Data.Ch0 = Yaw;
							Remote_Data.Ch1 = Pitch;
							flag = Data[6];
							shoot_flag = Data[7];
//							flag = 1;
//							Remote_Data.S1 = 1;
//							Remote_Data.S2 = 1;
				break;
								
      }
				
		}		 
}

/**
  * @brief  获得偏移角;标定正方向
  * @param  void
  * @retval 返回值单位“度”
  * @attention 获得枪口与底盘正方向的偏移角
  */
float GetDirAngle_Shoot(void)
{
    dir_angle_shoot = Holder.Yaw_0x20A.Position.Angle - bias_angle_shoot;
    dir_angle_shoot -= dir_angle_shoot>4096? 8192:0;
    dir_angle_shoot += dir_angle_shoot<-4096? 8192:0;
	  dir_angle_shoot = dir_angle_shoot * 360/8192; 
	  return dir_angle_shoot;
}

/* USER CODE END 1 */
