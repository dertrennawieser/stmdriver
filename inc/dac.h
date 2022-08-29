/*
 * dac.h
 *
 *  Created on: Aug 29, 2022
 *      Author: moritz
 */

#ifndef DAC_H_
#define DAC_H_

#include "stm32f3xx.h"

void dac_init();
void dac_write(uint16_t value);

#endif /* DAC_H_ */
