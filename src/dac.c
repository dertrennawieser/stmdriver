/*
 * dac.c
 *
 *  Created on: Aug 29, 2022
 *      Author: moritz
 */

#include "dac.h"


void dac_init()
{
	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_DAC1EN);

	// configure pin as analog
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER4, GPIO_MODER_MODER4_0 + GPIO_MODER_MODER4_1);

	// enable dac1 ch1
	SET_BIT(DAC1->CR, DAC_CR_EN1);
	// disable ch1 output buffer
	SET_BIT(DAC1->CR, DAC_CR_BOFF1);
}

void dac_write(uint16_t value)
{
	DAC1->DHR12R1 = value & 0xFFF;
}
