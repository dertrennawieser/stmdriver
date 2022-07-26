/*
 * tim.c
 *
 *  Created on: 21.07.2022
 *      Author: moritz
 */

#include "tim.h"


void tim3_pwminit(uint16_t psc, uint16_t arr)
{
	//enable Clock für Timer 3
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);

	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);


	// CH1 - PA6
	MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFRL6, 2<<GPIO_AFRL_AFSEL6_Pos);	//PA6 -> AF2 CH1
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER6, GPIO_MODER_MODER6_1);		//pin alternate function mode

	// Timer 3 channel 1 compare mode = PWM1, preload buffer enabled
	MODIFY_REG(TIM3->CCMR1, TIM_CCMR1_OC1M + TIM_CCMR1_OC1PE, TIM_CCMR1_OC1M_2 + TIM_CCMR1_OC1M_1 + TIM_CCMR1_OC1PE);

	// Enable compare output
	SET_BIT(TIM3->CCER, TIM_CCER_CC1E);


	// CH2 - PA7
	MODIFY_REG(GPIOA->AFR[0], GPIO_AFRL_AFRL7, 2<<GPIO_AFRL_AFSEL7_Pos);	//PA7 -> AF2 CH2
	MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER7, GPIO_MODER_MODER7_1);		//pin alternate function mode

	// Timer 3 channel 2 compare mode = PWM1, preload buffer enabled
	MODIFY_REG(TIM3->CCMR1, TIM_CCMR1_OC2M + TIM_CCMR1_OC2PE, TIM_CCMR1_OC2M_2 + TIM_CCMR1_OC2M_1 + TIM_CCMR1_OC2PE);

	// Enable compare output
	SET_BIT(TIM3->CCER, TIM_CCER_CC2E);


	// CH3 - PB0
	MODIFY_REG(GPIOB->AFR[0], GPIO_AFRL_AFRL0, 2<<GPIO_AFRL_AFSEL0_Pos);	//PB0 -> AF2 CH3
	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER0, GPIO_MODER_MODER0_1);		//pin alternate function mode

	// Timer 3 channel 3 compare mode = PWM1, preload buffer enabled
	MODIFY_REG(TIM3->CCMR2, TIM_CCMR2_OC3M + TIM_CCMR2_OC3PE, TIM_CCMR2_OC3M_2 + TIM_CCMR2_OC3M_1 + TIM_CCMR2_OC3PE);

	// Enable compare output
	SET_BIT(TIM3->CCER, TIM_CCER_CC3E);


	// CH4 - PB1
	MODIFY_REG(GPIOB->AFR[0], GPIO_AFRL_AFRL1, 2<<GPIO_AFRL_AFSEL1_Pos);	//PB1 -> AF2 CH3
	MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER1, GPIO_MODER_MODER1_1);		//pin alternate function mode

	// Timer 3 channel 4 compare mode = PWM1, preload buffer enabled
	MODIFY_REG(TIM3->CCMR2, TIM_CCMR2_OC4M + TIM_CCMR2_OC4PE, TIM_CCMR2_OC4M_2 + TIM_CCMR2_OC4M_1 + TIM_CCMR2_OC4PE);

	// Enable compare output
	SET_BIT(TIM3->CCER, TIM_CCER_CC4E);


	//invert
	//SET_BIT(TIM3->CCER, TIM_CCER_CC1P + TIM_CCER_CC2P + TIM_CCER_CC3P + TIM_CCER_CC4P);

	//auto reload register (max value)
	TIM3->ARR = arr;

	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0;

	/*
	//enable compare interrupt
	SET_BIT(TIM3->DIER, TIM_DIER_CC1IE);
	SET_BIT(TIM3->DIER, TIM_DIER_CC2IE);
	SET_BIT(TIM3->DIER, TIM_DIER_CC3IE);
	SET_BIT(TIM3->DIER, TIM_DIER_CC4IE);

	//enable update interrupt
	//SET_BIT(TIM3->DIER, TIM_DIER_UIE);
	*/

	//enable prescaler
	TIM3->PSC = psc-1;
	//generate update event -> load prescaler
	SET_BIT(TIM3->EGR, TIM_EGR_UG);

	//Timer enable auto-preload
	SET_BIT(TIM3->CR1, TIM_CR1_ARPE);
}

void tim2_init(uint16_t psc, uint16_t arr)
{
	//TIM2 & TIM4 clock enable
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);

	//auto reload register (max value)
	TIM2->ARR = arr;

	//Precsaler for 100MHz -> 1MHz (/100)
	TIM2->PSC = psc-1;

	//generate update event -> load prescaler
	SET_BIT(TIM2->EGR, TIM_EGR_UG);

	//enable update interrupt
	//SET_BIT(TIM2->DIER, TIM_DIER_UIE);
}

void tim4_init(uint16_t psc, uint16_t arr)
{
	//TIM2 & TIM4 clock enable
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM4EN);

	//auto reload register (max value)
	TIM4->ARR = arr;

	//Precsaler for 100MHz -> 1MHz (/100)
	TIM4->PSC = psc-1;

	//generate update event -> load prescaler
	SET_BIT(TIM4->EGR, TIM_EGR_UG);

	//enable update interrupt
	//SET_BIT(TIM4->DIER, TIM_DIER_UIE);
}

/*
void TIM3_IRQHandler()
{
	if(READ_BIT(TIM3->SR, TIM_SR_CC1IF))
	{
		CLEAR_BIT(TIM3->SR, TIM_SR_CC1IF);

	}
	else if(READ_BIT(TIM3->SR, TIM_SR_CC2IF))
	{
		CLEAR_BIT(TIM3->SR, TIM_SR_CC2IF);

	}
	else if(READ_BIT(TIM3->SR, TIM_SR_CC3IF))
	{
		CLEAR_BIT(TIM3->SR, TIM_SR_CC3IF);

	}
	else if(READ_BIT(TIM3->SR, TIM_SR_CC4IF))
	{
		CLEAR_BIT(TIM3->SR, TIM_SR_CC4IF);

	}
}
*/

