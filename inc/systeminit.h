/*
 * systeminit.h
 *
 *  Created on: 21.07.2022
 *      Author: moritz
 */

#ifndef SYSTEMINIT_H_
#define SYSTEMINIT_H_

#include "stm32l0xx.h"


void SystemInit();

void sleeponexit();
void stoponexit();

void wait_ms(uint32_t);
uint64_t Systick_GetTick();

#endif /* SYSTEMINIT_H_ */
