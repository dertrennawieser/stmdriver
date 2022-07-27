/*
 * adc.c
 *
 *  Created on: 21.07.2022
 *      Author: moritz
 */

#include "adc.h"

uint16_t data = 0;
bool new = false;

void adc_init()
{
	data = 0;

	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);

	// PA1 - Analog in 1
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER1, GPIO_MODER_MODER1_1 + GPIO_MODER_MODER1_0);
	// PA2 - Analog in 2
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER2, GPIO_MODER_MODER2_1 + GPIO_MODER_MODER2_0);

	// ADC CLK = PCLK2 / 8 = 12.5 MHz
	MODIFY_REG(ADC1_COMMON->CCR, ADC_CCR_ADCPRE, ADC_CCR_ADCPRE_0 + ADC_CCR_ADCPRE_1);

	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_ADC1EN);

	SET_BIT(ADC1->CR2, ADC_CR2_ADON);

	// 84 ADC clk cycles conversion time
	MODIFY_REG(ADC1->SMPR2, ADC_SMPR2_SMP0 + ADC_SMPR2_SMP1, ADC_SMPR2_SMP0_2 + ADC_SMPR2_SMP1_2);

	wait_ms(2);

	// enable EOC interrupt
	SET_BIT(ADC1->CR1, ADC_CR1_EOCIE);
	NVIC_EnableIRQ(ADC_IRQn);
}

void adc_startconversion(uint8_t ch)
{
	if(READ_BIT(ADC1->SR, ADC_SR_STRT))
		return;

	// convert 1 channel
	MODIFY_REG(ADC1->SQR1, ADC_SQR1_L, 0);

	// select channel
	MODIFY_REG(ADC1->SQR3, ADC_SQR3_SQ1, ch);

	// start conversion
	SET_BIT(ADC1->CR2, ADC_CR2_SWSTART);
}

uint16_t adc_getnewdata()
{
	if(new)
	{
		new = false;
		return data;
	}
	else
	{
		return -1;
	}
}

bool adc_busy()
{
	if(READ_BIT(ADC1->SR, ADC_SR_STRT))
		return true;
	else
		return false;
}

void ADC_IRQHandler()
{
	// clear ADC busy flag
	CLEAR_BIT(ADC1->SR, ADC_SR_STRT);

	// read DR (clears EOC)
	data = ADC1->DR;
	new = true;
}
