/*
 * i2cf303.c
 *
 *	I2C f�r STM32F303RE
 *
 *  Created on: 11.08.2020
 *      Author: Moritz
 */

#include "i2c.h"

typedef struct
{
	volatile I2C_TypeDef* reg_ptr;
	volatile uint8_t i2c_rxbufferspace[I2C_BUFFER_SIZE];
	volatile uint8_t* rxbuffer;
	volatile uint8_t* rx;
	volatile uint8_t* tx;
	volatile int txsize;
	volatile int rxsize;
	volatile int rxbuffersize;
}i2c_s;


i2c_s i2c1 = { I2C1 };
i2c_s i2c2 = { I2C2 };
i2c_s i2c3 = { I2C3 };

static void data_size(volatile I2C_TypeDef*, int);


static void I2C_Interrupt(i2c_s* i2c_ptr)
{
	volatile I2C_TypeDef* reg_ptr = i2c_ptr->reg_ptr;

	if(READ_BIT(reg_ptr->ISR, I2C_ISR_TXIS))
	{
		//ITM_SendString("TXIS\n");
		WRITE_REG(reg_ptr->TXDR, *(i2c_ptr->tx));
		i2c_ptr->tx++;
		i2c_ptr->txsize--;

		if (READ_BIT(reg_ptr->ISR, I2C_ISR_TCR))
			data_size(reg_ptr, i2c_ptr->txsize);

	}
	else if(READ_BIT(reg_ptr->ISR, I2C_ISR_RXNE))
	{
		//ITM_SendString("RXNE\n");
		*(i2c_ptr->rxbuffer) = READ_REG(reg_ptr->RXDR);
		//out((int)i2c_ptr->rxbuffer[i2c_ptr->i]);
		i2c_ptr->rxbuffer++;
		i2c_ptr->rxsize--;

		if (READ_BIT(reg_ptr->ISR, I2C_ISR_TCR))
			data_size(reg_ptr, i2c_ptr->rxsize);
	}
	else if(READ_BIT(reg_ptr->ISR, I2C_ISR_TC))
	{
		//ITM_SendString("TC\n");
		//i2c_ptr->i = 0;
		i2c_ptr->rxbuffer = i2c_ptr->i2c_rxbufferspace;
		if(i2c_ptr->rxsize>0)
		{
			SET_BIT(reg_ptr->CR2, I2C_CR2_RD_WRN);
			data_size(reg_ptr, i2c_ptr->rxsize);
			SET_BIT(reg_ptr->CR2, I2C_CR2_START);
		}
		else
			SET_BIT(reg_ptr->CR2, I2C_CR2_STOP);
	}
	else if(READ_BIT(reg_ptr->ISR, I2C_ISR_STOPF))
	{
		//ITM_SendString("STOPF\n");
		SET_BIT(reg_ptr->ICR, I2C_ICR_STOPCF);
		if(READ_BIT(reg_ptr->CR2, I2C_CR2_RD_WRN))
		{
			i2c_ptr->rxbuffer = i2c_ptr->i2c_rxbufferspace;
			for(int j=0; j<i2c_ptr->rxbuffersize; j++)
			{
				*(i2c_ptr->rx) = i2c_ptr->rxbuffer[j];
				i2c_ptr->rx++;
			}
			__NOP();
		}
	}
	else if(READ_BIT(reg_ptr->ISR, I2C_ISR_NACKF))
	{
		SET_BIT(reg_ptr->ICR, I2C_ICR_NACKCF);
		//ITM_SendString("NACK\n");
	}
}

void I2C_error(I2C_TypeDef* reg_ptr)
{
	if(READ_BIT(reg_ptr->ISR, I2C_ISR_BERR))
	{
		SET_BIT(reg_ptr->ICR, I2C_ICR_BERRCF);

	}
	else if(READ_BIT(reg_ptr->ISR, I2C_ISR_ARLO))
	{
		SET_BIT(reg_ptr->ICR, I2C_ICR_ARLOCF);

	}
	else if(READ_BIT(reg_ptr->ISR, I2C_ISR_OVR))
	{
		SET_BIT(reg_ptr->ICR, I2C_ICR_OVRCF);

	}
	else if(READ_BIT(reg_ptr->ISR, I2C_ISR_PECERR))
	{
		SET_BIT(reg_ptr->ICR, I2C_ICR_PECCF);

	}
	else if(READ_BIT(reg_ptr->ISR, I2C_ISR_TIMEOUT))
	{
		SET_BIT(reg_ptr->ICR, I2C_ICR_TIMOUTCF);

	}
	else if(READ_BIT(reg_ptr->ISR, I2C_ISR_ALERT))
	{
		SET_BIT(reg_ptr->ICR, I2C_ICR_ALERTCF);

	}
}

void I2C1_IRQHandler()
{
	I2C_Interrupt(&i2c1);
	I2C_error(I2C1);
}

void I2C2_IRQHandler()
{
	I2C_Interrupt(&i2c2);
	I2C_error(I2C2);
}

void I2C3_IRQHandler()
{
	I2C_Interrupt(&i2c3);
	I2C_error(I2C3);
}


static void data_size(volatile I2C_TypeDef* reg_ptr, int datasize)
{
	if (datasize>255)
	{
		MODIFY_REG(reg_ptr->CR2, I2C_CR2_NBYTES, 255 << I2C_CR2_NBYTES_Pos);
		//reload setzen f�r n>255
		SET_BIT(reg_ptr->CR2, I2C_CR2_RELOAD);
	}
	else
	{
		MODIFY_REG(reg_ptr->CR2, I2C_CR2_NBYTES, datasize << I2C_CR2_NBYTES_Pos);
		CLEAR_BIT(reg_ptr->CR2, I2C_CR2_RELOAD);
	}
}

/*
 * I2C peripheral busy indication
 * Parameter 1: I2C1, I2C2  (I2C3 for F303xD/E and F398xE devices only)
 * returns true if communication is in progress on the bus
 */
bool i2c_busy(I2C_TypeDef* reg_ptr)
{
	if(READ_BIT(reg_ptr->ISR, I2C_ISR_BUSY) || READ_BIT(reg_ptr->CR2, I2C_CR2_START))
		return true;
	else
		return false;
}

/*
 * I2C peripheral initialization
 * Parameter 1: I2C1, I2C2  (I2C3 for F303xD/E and F398xE devices only)
 * Parameter 2: true -> 400kHz	false -> 10kHz
 */
void i2c_init(I2C_TypeDef* reg_ptr, bool fastMode, uint32_t apb1_clock)
{
	if (reg_ptr==I2C1)
	{
		// Enable Port B
		SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN);

		// I2C1 PB8=SCL, alternate function 4 open-drain
		MODIFY_REG(GPIOB->AFR[1], GPIO_AFRH_AFSEL8, 4<<GPIO_AFRH_AFSEL8_Pos);
		MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE8, GPIO_MODER_MODE8_1);
		SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_8);

		// I2C1 PB9=SDA, alternate function 4 open-drain
		MODIFY_REG(GPIOB->AFR[1], GPIO_AFRH_AFSEL9, 4<<GPIO_AFRH_AFSEL9_Pos);
		MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE9, GPIO_MODER_MODE9_1);
		SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_9);

		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);

		NVIC_EnableIRQ(I2C1_IRQn);
	}

	if (reg_ptr==I2C2)
	{
		// Enable Port B
		SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN);

		// I2C2 PB10=SCL, alternate function 6 open-drain
		MODIFY_REG(GPIOB->AFR[1], GPIO_AFRH_AFSEL10, 6<<GPIO_AFRH_AFSEL10_Pos);
		MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE10, GPIO_MODER_MODE10_1);
		SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_10);

		// I2C2 PB11=SDA, alternate function 6 open-drain
		MODIFY_REG(GPIOB->AFR[1], GPIO_AFRH_AFSEL11, 6<<GPIO_AFRH_AFSEL11_Pos);
		MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE11, GPIO_MODER_MODE11_1);
		SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_11);

		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);

		NVIC_EnableIRQ(I2C2_IRQn);
	}

	if (reg_ptr==I2C3)
	{
		SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN);
		SET_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN);

		// I2C3 PA8=SCL, alternate function 7 open-drain
		MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL8, 7<<GPIO_AFRH_AFSEL8_Pos);
		MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODE8, GPIO_MODER_MODE8_1);
		SET_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_8);

		// I2C3 PB4=SDA, alternate function 7 open-drain
		MODIFY_REG(GPIOB->AFR[0], GPIO_AFRL_AFSEL4, 7<<GPIO_AFRL_AFSEL4_Pos);
		MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODE4, GPIO_MODER_MODE4_1);
		SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_4);

		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C3EN);

		NVIC_EnableIRQ(I2C3_IRQn);
	}

    // Disable the I2C peripheral
    CLEAR_BIT(reg_ptr->CR1, I2C_CR1_PE);

    // Configure timing
    if (fastMode)
	{
		// i2c clock must be <= 32 MHz otherwise the SCLDEL value would not fit into the register
		uint32_t prescaler=apb1_clock/8000000;
		if (prescaler<1)
			prescaler=1;

		uint32_t i2c_clock=apb1_clock/prescaler;
		MODIFY_REG(reg_ptr->TIMINGR, I2C_TIMINGR_PRESC,  (prescaler-1)         << I2C_TIMINGR_PRESC_Pos);
		MODIFY_REG(reg_ptr->TIMINGR, I2C_TIMINGR_SCLL,   (i2c_clock/800000-1)  << I2C_TIMINGR_SCLL_Pos);
		MODIFY_REG(reg_ptr->TIMINGR, I2C_TIMINGR_SCLH,   (i2c_clock/2000000-1) << I2C_TIMINGR_SCLH_Pos);
		MODIFY_REG(reg_ptr->TIMINGR, I2C_TIMINGR_SDADEL, (i2c_clock/4000000)   << I2C_TIMINGR_SDADEL_Pos); // no -1 on purpose
		MODIFY_REG(reg_ptr->TIMINGR, I2C_TIMINGR_SCLDEL, (i2c_clock/2000000-1) << I2C_TIMINGR_SCLDEL_Pos);
	}
	else
	{
		// i2c clock must be <= 12.8 MHz otherwise the SCLDEL value would not fit into the register
		uint32_t prescaler=apb1_clock/4000000;
		if (prescaler<1)
			prescaler=1;

		uint32_t i2c_clock=apb1_clock/prescaler;
		MODIFY_REG(reg_ptr->TIMINGR, I2C_TIMINGR_PRESC,  (prescaler-1)         << I2C_TIMINGR_PRESC_Pos);
		MODIFY_REG(reg_ptr->TIMINGR, I2C_TIMINGR_SCLL,   (i2c_clock/20000-1)  << I2C_TIMINGR_SCLL_Pos);
		MODIFY_REG(reg_ptr->TIMINGR, I2C_TIMINGR_SCLH,   (i2c_clock/20408-1)  << I2C_TIMINGR_SCLH_Pos);
		MODIFY_REG(reg_ptr->TIMINGR, I2C_TIMINGR_SDADEL, (i2c_clock/2000000)   << I2C_TIMINGR_SDADEL_Pos); // no -1 on purpose
		MODIFY_REG(reg_ptr->TIMINGR, I2C_TIMINGR_SCLDEL, (i2c_clock/800000-1)  << I2C_TIMINGR_SCLDEL_Pos);
	}

    // Stop and Restart will be generated by software
    CLEAR_BIT(reg_ptr->CR2, I2C_CR2_AUTOEND);

    // Enable the I2C peripheral
    SET_BIT(reg_ptr->CR1, I2C_CR1_PE);

    //Enable i2c event+error interrupts
    SET_BIT(reg_ptr->CR1, I2C_CR1_TCIE);
    SET_BIT(reg_ptr->CR1, I2C_CR1_STOPIE);
    SET_BIT(reg_ptr->CR1, I2C_CR1_NACKIE);
    SET_BIT(reg_ptr->CR1, I2C_CR1_RXIE);
    SET_BIT(reg_ptr->CR1, I2C_CR1_TXIE);
    SET_BIT(reg_ptr->CR1, I2C_CR1_ERRIE);

}

/*
 * Start I2C communication
 * Parameter 1: I2C1, I2C2  (I2C3 for F303xD/E and F398xE devices only)
 * Parameter 2: 7 bit slave address
 * Parameter 3: pointer to send data buffer (0 if unused)
 * Parameter 4: number of bytes to send
 * Parameter 5: pointer to receive data buffer (0 if unused)
 * Parameter 6:	number of bytes to receive
 */
void i2c_start(I2C_TypeDef* reg_ptr, uint8_t addr, uint8_t* send_buffer, int send_size, uint8_t* receive_buffer, int receive_size)
{
	if(READ_BIT(reg_ptr->ISR, I2C_ISR_BUSY))
		return;

	__disable_irq();

	i2c_s* i2c_ptr;

	if (reg_ptr==I2C1)
		i2c_ptr = &i2c1;
	if (reg_ptr==I2C2)
		i2c_ptr = &i2c2;
	if (reg_ptr==I2C3)
		i2c_ptr = &i2c3;

	i2c_ptr->txsize = send_size;
	i2c_ptr->tx = send_buffer;
	i2c_ptr->rxsize = receive_size;
	i2c_ptr->rxbuffersize = receive_size;
	i2c_ptr->rx = receive_buffer;
	i2c_ptr->rxbuffer = i2c_ptr->i2c_rxbufferspace;

	MODIFY_REG(reg_ptr->CR2, I2C_CR2_SADD, addr<<1);

	if(send_size>0)
	{
		//Master transmitter
		CLEAR_BIT(reg_ptr->CR2, I2C_CR2_RD_WRN);

		data_size(reg_ptr, send_size);
	}
	else if(send_size==0 && receive_size>0)
	{
		//Master receiver
		SET_BIT(reg_ptr->CR2, I2C_CR2_RD_WRN);

		data_size(reg_ptr, receive_size);
	}
	else
		return;

	__enable_irq();

	SET_BIT(reg_ptr->CR2, I2C_CR2_START);
}
