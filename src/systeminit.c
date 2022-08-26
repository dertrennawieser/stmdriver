/*
 * systeminit.c
 *
 *  Created on: 21.07.2022
 *      Author: moritz
 */

#include "systeminit.h"


uint32_t SystemCoreClock = 2097000;
uint32_t APB1Clock = 2097000;

volatile uint32_t systick_count = 0;
/*
// Change system clock to internal 16 MHz R/C oscillator
void SystemInit(void)
{
    // Enable HSI oscillator
    SET_BIT(RCC->CR, RCC_CR_HSION);

    // Wait until HSI oscillator is ready
    while(!READ_BIT(RCC->CR, RCC_CR_HSIRDY));

    // Switch to HSI oscillator
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI);

    // Wait until the switch is done
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI);

    // Flash latency 1 wait state
    SET_BIT(FLASH->ACR, FLASH_ACR_LATENCY);

    // Update variables
    SystemCoreClock=16000000;
    APB1Clock = 16000000;

    // Switch the MSI oscillator off
    CLEAR_BIT(RCC->CR, RCC_CR_MSION);

    // systick interrupt every ms
	SysTick_Config(SystemCoreClock/1000);
}
*/

// Change system clock to 16 MHz using external 8 MHz Crystal
// Called by Assembler startup code
void SystemInit()
{
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

    // Wait until the PLL is fully stopped
    while(READ_BIT(RCC->CR, RCC_CR_PLLRDY));

    // Flash latency 1 wait state
    SET_BIT(FLASH->ACR, FLASH_ACR_LATENCY);

    // Enable HSE oscillator
    SET_BIT(RCC->CR, RCC_CR_HSEON);

    // Wait until HSE oscillator is ready
    while(!READ_BIT(RCC->CR, RCC_CR_HSERDY));

    // 16 MHz using the 8 MHz HSE oscillator multiply by 4 divide by 2
    WRITE_REG(RCC->CFGR, RCC_CFGR_PLLSRC_HSE + RCC_CFGR_PLLMUL4 + RCC_CFGR_PLLDIV2);

    // Enable PLL
    SET_BIT(RCC->CR, RCC_CR_PLLON);

    // Wait until PLL is ready
    while(!READ_BIT(RCC->CR, RCC_CR_PLLRDY));

    // Select PLL as clock source
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

    // Switch the MSI oscillator off
    CLEAR_BIT(RCC->CR, RCC_CR_MSION);

    // Switch the HSI oscillator off
    CLEAR_BIT(RCC->CR, RCC_CR_HSION);
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
	SET_BIT(PWR->CR, PWR_CR_LPSDSR);

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
	SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_0);
	SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR_1);
	SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS_2);
	while(1);
}
