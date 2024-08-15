#ifndef __DMA_UNPACK_H
#define __DMA_UNPACK_H

#include "data_fifo.h"
#include "protocol.h"
#include "usart.h"

#define FRAME_HEADER_LEN 5
#define CMD_ID_LEN 2
#define FRAME_TAIL_LEN 2

#define SOF 0xA5

#define PROTOCAL_FRAME_MAX_SIZE 200

/* 裁判系统数据结构 帧头 命令码 数据流 帧尾 */
typedef __packed struct
{
    uint8_t sof;
    uint16_t data_len;
    uint8_t seq;
    uint8_t crc8;
} frame_header_t;

typedef enum
{
    STEP_HEADER_SOF = 0,
    STEP_LENGTH_LOW = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16 = 5,
} unpack_step_e; /* 数据流信息如帧头 0 */

typedef enum
{
    UART_IDLE_IT = 0,
    UART_DMA_HALF_IT = 1,
    UART_DMA_FULL_IT = 2,
} uart_it_type_e; /* 中断方式如中断 0 */

typedef struct
{
    fifo_s_t *p_fifo; /* DMA数据的起始地址 */
    frame_header_t *p_header;
    uint16_t data_len;
    uint8_t protocol_packet[PROTOCAL_FRAME_MAX_SIZE];
    unpack_step_e unpack_step;
    uint16_t index;
} unpack_data_t;

typedef struct
{
    DMA_HandleTypeDef *hdma_usart_rx;
    fifo_s_t *p_fifo;
    uint16_t buff_size;
    uint8_t *buff[2];
    uint16_t read_index;
    uint16_t write_index;
} uart_dma_rx_t;

void dma_buffer_to_unpack_buffer(uart_dma_rx_t *p_uart_dma, uart_it_type_e it_type);
void judge_unpack_fifo_data(unpack_data_t *p_obj, uint8_t sof);
//void judge_rx_handler(uint8_t *p_frame);
void data_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof);
uint8_t* protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf);
#endif
