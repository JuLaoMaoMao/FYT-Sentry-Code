/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
#define CMD_WARE 3 
/* USER CODE END Private defines */

void MX_UART4_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void UART1_RemoteStart(void);
void UART2_SendWave(uint8_t num, uint8_t bytes, ...);
void UART3_JudgeStart(void);
void UART4_IMUStart(void);
void UART6_IMUStart(void);

uint8_t __HAL_DMA_GET_CURRMEMTAR(DMA_HandleTypeDef *hdma);
uint16_t __HAL_DMA_GET_CURRDATCOUT(DMA_HandleTypeDef *hdma);

extern uint8_t uart1_buf[18];/* 遥控器 @定长18个8字节的数据 */
extern uint8_t uart3_dma_flag;
extern uint8_t uart3_idle_flag;
extern uint8_t uart3_m0_buf[1024];
extern uint8_t uart3_m1_buf[1024];
extern uint8_t uart3_tx_buf[120];
extern uint8_t uart6_rx_buf[200];

void uart1_dma_callback(DMA_HandleTypeDef *hdma);
void uart3_dma_m0_callback(DMA_HandleTypeDef *hdma);
void uart3_dma_m1_callback(DMA_HandleTypeDef *hdma);
void uart3_dma_m2_callback(DMA_HandleTypeDef *hdma);
extern DMA_HandleTypeDef hdma_usart3_rx;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

