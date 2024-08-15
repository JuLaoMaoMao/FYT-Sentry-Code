#include "dma_unpack.h"
#include "usart.h"
#include "Vision.h"
#include "string.h"
#include "Judge_Rx.h"

/* sof为固定值0xA5 */
/* 将unpack中的数据解析 */
void judge_unpack_fifo_data(unpack_data_t *p_obj, uint8_t sof)
{
    uint8_t byte = 0;

    while (fifo_used_count(p_obj->p_fifo))
    {
        byte = fifo_s_get(p_obj->p_fifo);
        switch (p_obj->unpack_step) /* 状态机 */
        {
        case STEP_HEADER_SOF:
        {
            if (byte == sof) /* 帧头数据中的SOF匹配 */
            {
                p_obj->unpack_step = STEP_LENGTH_LOW;
                p_obj->protocol_packet[p_obj->index++] = byte;
            }
            else
            {
                p_obj->index = 0;
            }
        }
        break;

        case STEP_LENGTH_LOW: /* 解析数据长度字节拼凑uint16_t */
        {
            p_obj->data_len = byte;
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_LENGTH_HIGH;
        }
        break;

        case STEP_LENGTH_HIGH: /* 解析高8位 */
        {
            p_obj->data_len |= (byte << 8);
            p_obj->protocol_packet[p_obj->index++] = byte;

            if (p_obj->data_len < (PROTOCAL_FRAME_MAX_SIZE - FRAME_HEADER_LEN - FRAME_TAIL_LEN)) //数据长度合理
            {
                p_obj->unpack_step = STEP_FRAME_SEQ; /* 继续解析 */
            }
            else /* 出错回到初始状态 */
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
            }
        }
        break;

        case STEP_FRAME_SEQ:
        {
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_HEADER_CRC8;
        }
        break;

        case STEP_HEADER_CRC8:
        {
            p_obj->protocol_packet[p_obj->index++] = byte;

            if (p_obj->index == FRAME_HEADER_LEN) /* 帧头数据长度正确 */
            {
                if (verify_crc8_check_sum(p_obj->protocol_packet, FRAME_HEADER_LEN))
                {
                    p_obj->unpack_step = STEP_DATA_CRC16;
                }
                else /* 不合理回归初始状态 */
                {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
            }
            else /* 不合理回归初始状态 */
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
            }
        }
        break;

        case STEP_DATA_CRC16:
        {
            if (p_obj->index < (FRAME_HEADER_LEN + CMD_ID_LEN +
                                p_obj->data_len + FRAME_TAIL_LEN)) /* 继续缓存包数据 */
            {
                p_obj->protocol_packet[p_obj->index++] = byte;
            }
            if (p_obj->index >= (FRAME_HEADER_LEN + CMD_ID_LEN +
                                 p_obj->data_len + FRAME_TAIL_LEN)) /* 全部缓存成功 */
            {
                /* 成功解析完成，回归初始状态 */
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
                /* 整包校检 */
                if (verify_crc16_check_sum(p_obj->protocol_packet, FRAME_HEADER_LEN + CMD_ID_LEN + p_obj->data_len + FRAME_TAIL_LEN))
                {
                    /* 解析数据 */
                    judge_rx_handler(p_obj->protocol_packet);
                }
            }
        }
        break;

        default:
        {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
        }
        break;
        }
    }
}

/* dma数据流转为fifo数据流格式 */
void get_dma_memory_msg(uart_dma_rx_t *p_uart_dma, uint8_t *mem_id, uint16_t *remain_cnt)
{
    *mem_id = __HAL_DMA_GET_CURRMEMTAR(p_uart_dma->hdma_usart_rx);
    *remain_cnt = __HAL_DMA_GET_CURRDATCOUT(p_uart_dma->hdma_usart_rx);
}

void dma_buffer_to_unpack_buffer(uart_dma_rx_t *p_uart_dma, uart_it_type_e it_type)
{
    int16_t tmp_len;
    uint8_t current_memory_id;
    uint16_t remain_data_counter;
    uint8_t *pdata = p_uart_dma->buff[0];

    get_dma_memory_msg(p_uart_dma, &current_memory_id, &remain_data_counter);
    /* 不知道有什么区别反正原来的不行，按道理来说满了过后remain_data_counter就应该是0，先这样吧 */
    if (UART_IDLE_IT == it_type)
    {
        if (current_memory_id)
        {
            p_uart_dma->write_index = p_uart_dma->buff_size * 2- remain_data_counter;
        }
        else
        {
            p_uart_dma->write_index = p_uart_dma->buff_size - remain_data_counter;
        }
    }
    else if (UART_DMA_FULL_IT == it_type)
    {
        if (current_memory_id)
        {
            p_uart_dma->write_index = p_uart_dma->buff_size * 2 - remain_data_counter;
        }
        else
        {
            p_uart_dma->write_index = p_uart_dma->buff_size  - remain_data_counter;
        }
    }

    if (p_uart_dma->write_index < p_uart_dma->read_index)
    {
        tmp_len = p_uart_dma->buff_size * 2 - p_uart_dma->read_index;
        fifo_s_puts(p_uart_dma->p_fifo, &pdata[p_uart_dma->read_index], tmp_len);
        p_uart_dma->read_index = 0;

        tmp_len = p_uart_dma->write_index;
        fifo_s_puts(p_uart_dma->p_fifo, &pdata[p_uart_dma->read_index], tmp_len);
        p_uart_dma->read_index = p_uart_dma->write_index;
    }
    else
    {
        tmp_len = p_uart_dma->write_index - p_uart_dma->read_index;

        fifo_s_puts(p_uart_dma->p_fifo, &pdata[p_uart_dma->read_index], tmp_len);

        p_uart_dma->read_index = (p_uart_dma->write_index) % (p_uart_dma->buff_size * 2);
    }
}

/**
  * @brief  裁判系统发送数据打包.
  * @param  p_data 发送数据段地址
			cmd_id 命令码ID
			sof		数据帧起始字节，固定值为 0xA5
  * @note   发送数据->发送给fifo中，含头帧尾帧CRC等的添加.
  * @retval NULL.
  */
void data_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof)
{
    uint8_t tx_buf[PROTOCAL_FRAME_MAX_SIZE];

    uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;

    //尾部追加CRC校验
    protocol_packet_pack(cmd_id, p_data, len, sof, tx_buf);

    //打包放进fifo中

    memcpy(uart3_tx_buf, tx_buf, frame_length);
    return ;
}

/**
  * @brief  数据包尾部追加CRC.
  * @param
* @note   内部调用.
  * @retval NULL.
  */
uint8_t* protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf)
{
    static uint8_t seq;

    uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;
    frame_header_t *p_header = (frame_header_t*)tx_buf;

    p_header->sof          = sof;
    p_header->data_len     = len;

    if (sof == UP_REG_ID)
    {
        if (seq++ >= 255)
            seq = 0;

        p_header->seq = seq;
    }
    else
    {
        p_header->seq = 0;
    }

    memcpy(&tx_buf[HEADER_LEN], (uint8_t*)&cmd_id, CMD_LEN);//复制帧头数据段
    append_crc8_check_sum(tx_buf, HEADER_LEN);//追加帧头的CRC校验
    memcpy(&tx_buf[HEADER_LEN + CMD_LEN], p_data, len);//复制数据段
    append_crc16_check_sum(tx_buf, frame_length);//追加尾帧的CRC校验

    return tx_buf;
}


