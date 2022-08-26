/*
 * systeminit.c
 *
 *  Created on: 21.07.2022
 *      Author: moritz
 */

#include "systeminit.h"

uint32_t SystemCoreClock=16000000;
uint32_t APB1Clock=16000000;

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

	//FPU einschalten
	SCB->CPACR = 0x00F00000;
}

void sleeponexit()
{
	// disable deepsleep mode
	CLEAR_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);

	// enable sleep on exit
	SET_BIT(SCB->SCR, SCB_SCR_SLEEPONEXIT_Msk);
}

void stoponexit()
{
	// set v reg low power mode when cpu enters deepsleep
	SET_BIT(PWR->CR, PWR_CR_LPDS);

	// enter stop mode when cpu enters deepsleep
	CLEAR_BIT(PWR->CR, PWR_CR_PDDS);

	// clear wake up flag
	SET_BIT(PWR->CR, PWR_CR_CWUF);

	// enable deepsleep mode
	SET_BIT(SCB->SCR, SCB_SCR_SLEEPDEEP_Msk);

	// enable sleep on exit
	SET_BIT(SCB->SCR, SCB_SCR_SLEEPONEXIT_Msk);
}

void wait_ms(uint32_t ticks) {
	uint64_t end = systick_count + ticks;
	while (systick_count < end);
}

uint64_t systick_gettick() {
	return systick_count;
}

void SysTick_Handler(void)
{
    systick_count++;
}

void iwdg_init()
{
	// disable iwdg in debug mode
	SET_BIT(DBGMCU->APB1FZ, DBGMCU_APB1_FZ_DBG_IWDG_STOP);

	// enable iwdg
	IWDG->KR = 0x0000CCCC;

	// enable register write access
	IWDG->KR = 0x00005555;

	// set prescaler
	IWDG->PR = 0;

	// set reload value
	IWDG->RLR = 600;

	// wait for the registers to be updated
	uint32_t start = systick_gettick();
	while(IWDG->SR)
	{
		if((systick_gettick() - start) > TIMEOUT_MS)
			fault_handler();
	}

	// set window and refresh iwdg
	IWDG->KR = 0x0000AAAA;
}

void iwdg_refresh()
{
	IWDG->KR = 0x0000AAAA;
}

void fault_handler()
{

	while(1);
}
