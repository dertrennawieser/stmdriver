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

	SET_BIT(RCC->AHBENR, RCC_AHBENR_GPIOAEN);

	// PA1 - Analog in 1
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER1, GPIO_MODER_MODER1_0 + GPIO_MODER_MODER1_1);

	// Enable clock for ADC
	    SET_BIT(RCC->AHBENR, RCC_AHBENR_ADC12EN);

	    // Disable the ADC
	    if (READ_BIT(ADC1->ISR, ADC_ISR_ADRDY))
	    {
	        SET_BIT(ADC1->ISR, ADC_ISR_ADRDY);
	    }
	    if (READ_BIT(ADC1->CR, ADC_CR_ADEN))
	    {
	        SET_BIT(ADC1->CR, ADC_CR_ADDIS);
	    }

	    // Wait until ADC is disabled
	    while (READ_BIT(ADC1->CR, ADC_CR_ADEN));

	    // Enable ADC voltage regulator
	    MODIFY_REG(ADC1->CR, ADC_CR_ADVREGEN, 0);
	    MODIFY_REG(ADC1->CR, ADC_CR_ADVREGEN, ADC_CR_ADVREGEN_0);

	    // Delay 1-2 ms
	    wait_ms(2);

	    // ADC Clock = HCLK/4
	    MODIFY_REG(ADC12_COMMON->CCR, ADC12_CCR_CKMODE, ADC12_CCR_CKMODE_0 + ADC12_CCR_CKMODE_1);

	    // Single ended mode for all channels
	    WRITE_REG(ADC1->DIFSEL,0);

	    // Start calibration for single ended mode
	    CLEAR_BIT(ADC1->CR, ADC_CR_ADCALDIF);
	    SET_BIT(ADC1->CR, ADC_CR_ADCAL);

	    // Wait until the calibration is finished
	    while (READ_BIT(ADC1->CR, ADC_CR_ADCAL));

	    // Clear the ready flag
	    SET_BIT(ADC1->ISR, ADC_ISR_ADRDY);

	    // Enable the ADC repeatedly until success (workaround from errata)
	    do
	    {
	        SET_BIT(ADC1->CR, ADC_CR_ADEN);
	    }
	    while (!READ_BIT(ADC1->ISR, ADC_ISR_ADRDY));

	    // Select software start trigger
	    MODIFY_REG(ADC1->CFGR, ADC_CFGR_EXTEN, 0);

	    // Select single conversion mode
	    CLEAR_BIT(ADC1->CFGR, ADC_CFGR_CONT);

	    // Set sample time to 32 cycles
	    MODIFY_REG(ADC1->SMPR1, ADC_SMPR1_SMP1, ADC_SMPR1_SMP1_2);

	wait_ms(2);

	// enable EOC interrupt
	SET_BIT(ADC1->IER, ADC_IER_EOCIE);
	NVIC_SetPriority(ADC1_2_IRQn, 0);
	NVIC_EnableIRQ(ADC1_2_IRQn);
}

void adc_startconversion(uint8_t ch)
{
	if(READ_BIT(ADC1->CR, ADC_CR_ADSTART))
		return;

	// convert 1 channel
	MODIFY_REG(ADC1->SQR1, ADC_SQR1_L, 0);

	// select channel
    MODIFY_REG(ADC1->SQR1, ADC_SQR1_SQ1, ch<<ADC_SQR1_SQ1_Pos);

	// start conversion
    SET_BIT(ADC1->CR, ADC_CR_ADSTART);
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
	if(READ_BIT(ADC1->CR, ADC_CR_ADSTART))
		return true;
	else
		return false;
}

void ADC1_2_IRQHandler()
{
	// clear ADC busy flag
	CLEAR_BIT(ADC1->CR, ADC_CR_ADSTART);

	// read DR (clears EOC)
	data = ADC1->DR;
	new = true;
}
