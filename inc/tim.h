/*
 * tim.h
 *
 *  Created on: 21.07.2022
 *      Author: moritz
 */

#ifndef TIM_H_
#define TIM_H_

#include "stm32f4xx.h"

void tim3_pwminit(uint16_t psc, uint16_t arr);

void tim2_init(uint16_t psc);
void tim4_init(uint16_t psc);

#endif /* TIM_H_ */
