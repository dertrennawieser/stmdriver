/*
 * rtc.c
 *
 *  Created on: 28.07.2022
 *      Author: moritz
 */

#include "rtc.h"


void rtc_init()
{
	// Enable the power interface
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);

	// Enable access to the backup domain
	SET_BIT(PWR->CR, PWR_CR_DBP);

	// Enable LSE oscillator with medium driver power
	MODIFY_REG(RCC->CSR, RCC_CSR_LSEDRV, RCC_CSR_LSEDRV_1);
	SET_BIT(RCC->CSR, RCC_CSR_LSEON);

	// Wait until LSE oscillator is ready
	while(!READ_BIT(RCC->CSR, RCC_CSR_LSERDY)) {}

	// Select LSE as clock source for the RTC
	MODIFY_REG(RCC->CSR, RCC_CSR_RTCSEL, RCC_CSR_RTCSEL_LSE);

	// Enable the RTC
	SET_BIT(RCC->CSR, RCC_CSR_RTCEN);
}

void rtc_periodicwakeup(uint16_t sec)
{
	// Unlock the write protection
	WRITE_REG(RTC->WPR, 0xCA);
	WRITE_REG(RTC->WPR, 0x53);

	// Stop the wakeup timer to allow configuration update
	CLEAR_BIT(RTC->CR, RTC_CR_WUTE);

	// Wait until the wakeup timer is ready for configuration update
	while (!READ_BIT(RTC->ISR, RTC_ISR_WUTWF));

	// Clock source of the wakeup timer is 1 Hz
	MODIFY_REG(RTC->CR, RTC_CR_WUCKSEL, RTC_CR_WUCKSEL_2);

	// The wakeup period is 0+1 clock pulses
	WRITE_REG(RTC->WUTR,sec-1);

	// Enable the wakeup timer with interrupts
	SET_BIT(RTC->CR, RTC_CR_WUTE + RTC_CR_WUTIE);

	// Switch the write protection back on
	WRITE_REG(RTC->WPR, 0xFF);

	// Enable EXTI20 interrupt on rising edge
	SET_BIT(EXTI->IMR, EXTI_IMR_IM20);
	SET_BIT(EXTI->RTSR, EXTI_RTSR_TR20);
	NVIC_EnableIRQ(RTC_IRQn);

	// Clear (old) pending interrupt flag
	CLEAR_BIT(RTC->ISR, RTC_ISR_WUTF);  // Clear in RTC
	SET_BIT(EXTI->PR, EXTI_PR_PR20);    // Clear in NVIC
}

