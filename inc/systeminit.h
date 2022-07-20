#ifndef SYSTEMINIT_H_
#define SYSTEMINIT_H_

#include "stm32f4xx.h"

void wait_ms(uint32_t);
uint64_t Systick_GetTick();

#endif /* SYSTEMINIT_H_ */
