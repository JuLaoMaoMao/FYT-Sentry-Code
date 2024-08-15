/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of?
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.? See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file data_fifo.c
 *  @version 1.0
 *  @date June 2017
 *
 *  @brief  genernal FIFO model interface for any data element type.
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "data_fifo.h"
#include "stdlib.h"
#include "stdio.h"

fifo_s_t *fifo_s_create(uint32_t unit_cnt)
{
    fifo_s_t *pfifo = NULL;
    uint8_t *base_addr = NULL;

    ASSERT(unit_cnt != 0); /* Check input parameters. */

    pfifo = (fifo_s_t *)malloc(sizeof(fifo_s_t)); /* Allocate Memory for pointer of new FIFO Control Block. */
    if (pfifo == NULL)
    {
        return NULL; /* Allocate Failure, exit now. */
    }
    base_addr = malloc(unit_cnt); /* Allocate memory for FIFO. */
    if (base_addr == NULL)
    {
        return NULL; /* Allocate Failure, exit now. */
    }
    fifo_s_init(pfifo, base_addr, unit_cnt);
    return pfifo;
}

void fifo_s_destory(fifo_s_t *pfifo)
{
    ASSERT(pfifo != NULL); /* Check input parameters. */
    ASSERT(pfifo->start_addr != NULL);

    free(pfifo->start_addr); /* free FIFO memory */

    free(pfifo); /* free FIFO Control Block memory. */
}

/*
    @param:
        unit_cnt:fifo的长度，单位长度为u8
*/
int32_t fifo_s_init(fifo_s_t *pfifo, void *base_addr, uint32_t unit_cnt)
{
    ASSERT(pfifo != NULL); /* Check input parameters. */
    ASSERT(base_addr != NULL);
    ASSERT(unit_cnt != 0);

    pfifo->start_addr = (uint8_t *)base_addr;
    pfifo->end_addr = (uint8_t *)base_addr + unit_cnt - 1;
    pfifo->buf_size = unit_cnt;
    pfifo->free = unit_cnt;
    pfifo->used = 0;
    pfifo->read_index = 0;
    pfifo->write_index = 0;
    return 0;
}

int32_t fifo_s_put(fifo_s_t *pfifo, uint8_t element)
{

    ASSERT(pfifo != NULL); /* Check input parameters. */

    if (pfifo->free <= 0)
    {
        return -1; /* Error, FIFO is full! */
    }

    pfifo->start_addr[pfifo->write_index++] = element;
    pfifo->write_index %= pfifo->buf_size;
    pfifo->free--;
    pfifo->used++;

    return 0;
}

/*
    func:
        向fifo中写入一定长度的数据
    @param: 
        pfifo:fifo结构体
        psouse:数据地址
        number:数据个数
    @return:
        puts_num:fifo成功记录的数据
*/
int32_t fifo_s_puts(fifo_s_t *pfifo, uint8_t *psource, uint32_t number)
{
    int puts_num = 0;
    uint32_t i;

    ASSERT(pfifo != NULL); /* Check input parameters. */

    if (psource == NULL)
        return -1;

    for (i = 0; (i < number) && (pfifo->free > 0); i++)
    {
        pfifo->start_addr[pfifo->write_index++] = psource[i]; /* 从write_index处记录数据 */
        pfifo->write_index %= pfifo->buf_size;
        pfifo->free--; /* 剩余个数 */
        pfifo->used++; /* 已使用个数 */
        puts_num++;
    }

    return puts_num;
}

uint8_t fifo_s_get(fifo_s_t *pfifo)
{
    uint8_t retval = 0;

    ASSERT(NULL != pfifo); /* Check input parameters. */

    retval = pfifo->start_addr[pfifo->read_index++];
    pfifo->read_index %= pfifo->buf_size;
    pfifo->free++;
    pfifo->used--;

    return retval;
}

uint16_t fifo_s_gets(fifo_s_t *pfifo, uint8_t *source, uint8_t len)
{
    uint8_t retval = 0;
    int i;

    ASSERT(pfifo != NULL); /* Check input parameters. */

    for (i = 0; (i < len) && (pfifo->used > 0); i++)
    {
        source[i] = pfifo->start_addr[pfifo->read_index++];
        pfifo->read_index %= pfifo->buf_size;
        pfifo->free++;
        pfifo->used--;
        retval++;
    }

    return retval;
}

/******************************************************************************************
//
//! \brief  Pre-Read an element from FIFO(in single mode).
//!
//! \param  [in]  pfifo is the pointer of valid FIFO.
//! \param  [in]  offset is the offset from current pointer.
//!
//! \retval the data element of FIFO.
//
******************************************************************************************/
uint8_t fifo_s_pre_read(fifo_s_t *pfifo, uint8_t offset)
{
    uint32_t index;

    ASSERT(pfifo != NULL); /* Check input parameters. */

    if (offset > pfifo->used)
    {
        return 0x00;
    }
    else
    {
        index = ((pfifo->read_index + offset) % pfifo->buf_size);
        return pfifo->start_addr[index]; /* Move Read Pointer to right position */
    }
}

/******************************************************************************************
//!
//! \retval - None-zero(true) if empty.
//!         - Zero(false) if not empty.
//
******************************************************************************************/
uint8_t fifo_is_empty(fifo_s_t *pfifo)
{

    ASSERT(pfifo != NULL); /* Check input parameter.*/

    return (pfifo->used == 0);
}

/*****************************************************************************************
//!
//! \retval - None-zero(true) if full.
//!         - Zero(false) if not full.
//
*****************************************************************************************/
uint8_t fifo_is_full(fifo_s_t *pfifo)
{

    ASSERT(pfifo != NULL); /* Check input parameter. */

    return (0 == pfifo->free);
}

/******************************************************************************************
//!
//! \retval The number of elements in FIFO.
//
******************************************************************************************/
uint32_t fifo_used_count(fifo_s_t *pfifo)
{
    ASSERT(pfifo != NULL); /* Check input parameter. */

    return (pfifo->used);
}

/******************************************************************************************
//!
//! \retval The number of elements in FIFO.
//
******************************************************************************************/
uint32_t fifo_free_count(fifo_s_t *pfifo)
{
    ASSERT(NULL != pfifo); /* Check input parameter. */

    return (pfifo->free);
}

/******************************************************************************************
//
//! \brief  Flush the content of FIFO.
//!
//! \param  [in] pfifo is the pointer of valid FIFO.
//!
//! \retval 0 if success, -1 if failure.
//
******************************************************************************************/
uint8_t fifo_flush(fifo_s_t *pfifo)
{

    ASSERT(NULL != pfifo); /* Check input parameters. */

    pfifo->free = pfifo->buf_size; /* Initialize FIFO Control Block. */
    pfifo->used = 0;
    pfifo->read_index = 0;
    pfifo->write_index = 0;

    return 0;
}

//*********************my_supplement**********************

#define FIFO_LENGTH_TEMP 100
void fifo_trans(fifo_s_t *pfifo1, fifo_s_t *pfifo2)
{
    uint8_t content[FIFO_LENGTH_TEMP];
    uint8_t length;

    length = fifo_s_gets(pfifo2, content, FIFO_LENGTH_TEMP); /* 读取原fifo中的内容 */
    fifo_s_puts(pfifo1, content, length);                    /* 写入目标fifo中 */
}
