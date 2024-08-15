#include "dma_unpack.h"
#include "usart.h"
#include "Vision.h"
#include "string.h"
#include "Judge_Rx.h"

/* sofΪ�̶�ֵ0xA5 */
/* ��unpack�е����ݽ��� */
void judge_unpack_fifo_data(unpack_data_t *p_obj, uint8_t sof)
{
    uint8_t byte = 0;

    while (fifo_used_count(p_obj->p_fifo))
    {
        byte = fifo_s_get(p_obj->p_fifo);
        switch (p_obj->unpack_step) /* ״̬�� */
        {
        case STEP_HEADER_SOF:
        {
            if (byte == sof) /* ֡ͷ�����е�SOFƥ�� */
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

        case STEP_LENGTH_LOW: /* �������ݳ����ֽ�ƴ��uint16_t */
        {
            p_obj->data_len = byte;
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_LENGTH_HIGH;
        }
        break;

        case STEP_LENGTH_HIGH: /* ������8λ */
        {
            p_obj->data_len |= (byte << 8);
            p_obj->protocol_packet[p_obj->index++] = byte;

            if (p_obj->data_len < (PROTOCAL_FRAME_MAX_SIZE - FRAME_HEADER_LEN - FRAME_TAIL_LEN)) //���ݳ��Ⱥ���
            {
                p_obj->unpack_step = STEP_FRAME_SEQ; /* �������� */
            }
            else /* ����ص���ʼ״̬ */
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

            if (p_obj->index == FRAME_HEADER_LEN) /* ֡ͷ���ݳ�����ȷ */
            {
                if (verify_crc8_check_sum(p_obj->protocol_packet, FRAME_HEADER_LEN))
                {
                    p_obj->unpack_step = STEP_DATA_CRC16;
                }
                else /* ������ع��ʼ״̬ */
                {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
            }
            else /* ������ع��ʼ״̬ */
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
            }
        }
        break;

        case STEP_DATA_CRC16:
        {
            if (p_obj->index < (FRAME_HEADER_LEN + CMD_ID_LEN +
                                p_obj->data_len + FRAME_TAIL_LEN)) /* ������������� */
            {
                p_obj->protocol_packet[p_obj->index++] = byte;
            }
            if (p_obj->index >= (FRAME_HEADER_LEN + CMD_ID_LEN +
                                 p_obj->data_len + FRAME_TAIL_LEN)) /* ȫ������ɹ� */
            {
                /* �ɹ�������ɣ��ع��ʼ״̬ */
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
                /* ����У�� */
                if (verify_crc16_check_sum(p_obj->protocol_packet, FRAME_HEADER_LEN + CMD_ID_LEN + p_obj->data_len + FRAME_TAIL_LEN))
                {
                    /* �������� */
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

/* dma������תΪfifo��������ʽ */
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
    /* ��֪����ʲô������ԭ���Ĳ��У���������˵���˹���remain_data_counter��Ӧ����0���������� */
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
  * @brief  ����ϵͳ�������ݴ��.
  * @param  p_data �������ݶε�ַ
			cmd_id ������ID
			sof		����֡��ʼ�ֽڣ��̶�ֵΪ 0xA5
  * @note   ��������->���͸�fifo�У���ͷ֡β֡CRC�ȵ����.
  * @retval NULL.
  */
void data_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof)
{
    uint8_t tx_buf[PROTOCAL_FRAME_MAX_SIZE];

    uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;

    //β��׷��CRCУ��
    protocol_packet_pack(cmd_id, p_data, len, sof, tx_buf);

    //����Ž�fifo��

    memcpy(uart3_tx_buf, tx_buf, frame_length);
    return ;
}

/**
  * @brief  ���ݰ�β��׷��CRC.
  * @param
* @note   �ڲ�����.
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

    memcpy(&tx_buf[HEADER_LEN], (uint8_t*)&cmd_id, CMD_LEN);//����֡ͷ���ݶ�
    append_crc8_check_sum(tx_buf, HEADER_LEN);//׷��֡ͷ��CRCУ��
    memcpy(&tx_buf[HEADER_LEN + CMD_LEN], p_data, len);//�������ݶ�
    append_crc16_check_sum(tx_buf, frame_length);//׷��β֡��CRCУ��

    return tx_buf;
}


