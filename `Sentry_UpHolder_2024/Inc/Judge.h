#ifndef _JUDGE_H
#define _JUDGE_H

#include "Judge_Rx.h"
#include "dma_unpack.h"
#include "usart.h"

void Judge_InitData(void);
void Judge_Proccess(void);
void Judge_SendData(void);
/* Ä¦²ÁÂÖµÈ¼¶ */
extern uint8_t judge_lid_state;

/* Ğ¡ÍÓÂİ×´Ì¬ */
extern uint8_t judge_spin_state;

/*´ò·û×´Ì¬*/
extern uint8_t judge_vision_state;
#endif
