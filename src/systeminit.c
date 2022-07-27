/*
 * systeminit.c
 *
 *  Created on: 21.07.2022
 *      Author: moritz
 */

#include "systeminit.h"

extern uint32_t SystemCoreClock;
extern uint32_t APB1Clock;

volatile uint32_t systick_count = 0;
/*
// Change system clock to 100 MHz using external 25 MHz crystal
// Called by Assembler startup code
void SystemInit(void)
{
	// Because the debugger switches PLL on, we may need to switch
	// back to the HSI oscillator before we can configure the PLL

	// Enable HSI oscillator
	SET_BIT(RCC->CR, RCC_CR_HSION);

	// Wait until HSI oscillator is ready
	while(!READ_BIT(RCC->CR, RCC_CR_HSIRDY));

	// Switch to HSI oscillator
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI);

	// Wait until the switch is done
	while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI);

	// Disable the PLL
	CLEAR_BIT(RCC->CR, RCC_CR_PLLON);

	// Wait until PLL is fully stopped
	while(READ_BIT(RCC->CR, RCC_CR_PLLRDY));

	// Voltage regulator scale 1
	MODIFY_REG(PWR->CR, PWR_CR_VOS, PWR_CR_VOS_0 + PWR_CR_VOS_1);

	// Flash latency 3 wait states
	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_3WS);

	// Enable HSE oscillator
	SET_BIT(RCC->CR, RCC_CR_HSEON);

	// Wait until HSE oscillator is ready
	while(!READ_BIT(RCC->CR, RCC_CR_HSERDY));

	// lowspeed I/O runs at 50 MHz
	WRITE_REG(RCC->CFGR, RCC_CFGR_PPRE1_DIV2);

	// 100 MHz using 25 MHz crystal with HSE/14 * 112/2
	WRITE_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLSRC_HSE + RCC_PLLCFGR_PLLM_3 + RCC_PLLCFGR_PLLM_2 + RCC_PLLCFGR_PLLM_1 + RCC_PLLCFGR_PLLN_4 + RCC_PLLCFGR_PLLN_5 + RCC_PLLCFGR_PLLN_6);

	// Enable PLL
	SET_BIT(RCC->CR, RCC_CR_PLLON);

	// Wait until PLL is ready
	while(!READ_BIT(RCC->CR, RCC_CR_PLLRDY));

	// Select PLL as clock source
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

	// Update variables
	APB1Clock=50000000;
	SystemCoreClock=100000000;

	// Disable the HSI oscillator
	CLEAR_BIT(RCC->CR, RCC_CR_HSION);

	//FPU einschalten
	SCB->CPACR = 0x00F00000;

	//systick interrupt every ms
	SysTick_Config(SystemCoreClock/1000);
}
*/

// Change system clock to 100 MHz using internal 16 MHz R/C oscillator
// Called by Assembler startup code
void SystemInit(void)
{
	// Switch to HSI oscillator
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI);

	// Wait until the switch is done
	while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI);

	// Disable the PLL, then we can configure it
	CLEAR_BIT(RCC->CR, RCC_CR_PLLON);

	// Flash latency 3 wait states
	MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_3WS);

	//lowspeed I/O runs at 50 MHz
	WRITE_REG(RCC->CFGR, RCC_CFGR_PPRE1_DIV2);

	// 100 MHz using the 16 MHz HSI oscillator with HSI/8 * 100/2
	WRITE_REG(RCC->PLLCFGR, RCC_PLLCFGR_PLLM_3 + RCC_PLLCFGR_PLLN_2 + RCC_PLLCFGR_PLLN_5 + RCC_PLLCFGR_PLLN_6);

	// Enable PLL
	SET_BIT(RCC->CR, RCC_CR_PLLON);

	// Wait until PLL is ready
	while(!READ_BIT(RCC->CR, RCC_CR_PLLRDY));

	// Select PLL as clock source
	MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

	// Update variable
	SystemCoreClock=100000000;
	APB1Clock=50000000;

	//FPU einschalten
	SCB->CPACR = 0x00F00000;

	//systick interrupt every ms
	SysTick_Config(SystemCoreClock/1000);
}

void wait_ms(uint32_t ticks) {
	uint64_t end = systick_count + ticks;
	while (systick_count < end);
}

uint64_t Systick_GetTick() {
	return systick_count;
}

void SysTick_Handler(void)
{
    systick_count++;
}
