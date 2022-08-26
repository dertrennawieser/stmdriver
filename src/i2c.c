/*
 * i2cf411.c
 *
 *  Created on: 27.03.2021
 *      Author: moritz
 */


#include "i2c.h"
#include "itm.h"


typedef struct
{
	volatile I2C_TypeDef* reg_ptr;
	volatile uint8_t saddr;
	volatile uint8_t rxbuffer[I2C_BUFFER_SIZE];
	volatile uint8_t i;
	volatile uint8_t* rx;
	volatile uint8_t* tx;
	volatile uint8_t txsize;
	volatile uint8_t rxsize;
	volatile uint8_t rxbuffersize;
}i2c_s;


i2c_s i2c1 = { I2C1 };
i2c_s i2c2 = { I2C2 };
i2c_s i2c3 = { I2C3 };


static void I2C_Interrupt(i2c_s* i2c_ptr)
{
	volatile I2C_TypeDef* reg_ptr = i2c_ptr->reg_ptr;
	if(READ_BIT(reg_ptr->SR1, I2C_SR1_SB))
	{
		SET_BIT(reg_ptr->CR1, I2C_CR1_ACK);
		if(i2c_ptr->txsize==0 && i2c_ptr->rxsize>0)
			WRITE_REG(reg_ptr->DR, (i2c_ptr->saddr<<1)+1);		//adr+lesen
		else
			WRITE_REG(reg_ptr->DR, i2c_ptr->saddr<<1);			//adr+schreiben
	}
	else if(READ_BIT(reg_ptr->SR1, I2C_SR1_ADDR))
	{
		READ_REG(reg_ptr->SR2);
		if(i2c_ptr->rxsize==1 && i2c_ptr->txsize==0)
		{
			CLEAR_BIT(reg_ptr->CR1, I2C_CR1_ACK);
			SET_BIT(reg_ptr->CR1, I2C_CR1_STOP);
		}
	}
	else if(READ_BIT(reg_ptr->SR1, I2C_SR1_TXE))
	{
		if(i2c_ptr->txsize > 0)
		{
			WRITE_REG(reg_ptr->DR, *(i2c_ptr->tx));
			i2c_ptr->tx++;
			i2c_ptr->txsize--;
		}
		else
		{
			if(i2c_ptr->rxsize > 0)
				SET_BIT(reg_ptr->CR1, I2C_CR1_START);
			else
				SET_BIT(reg_ptr->CR1, I2C_CR1_STOP);
		}
	}
	else if(READ_BIT(reg_ptr->SR1, I2C_SR1_RXNE))
	{
		if(i2c_ptr->rxsize > 0)
		{
			if(i2c_ptr->rxsize==2)
			{
				CLEAR_BIT(reg_ptr->CR1, I2C_CR1_ACK);
				SET_BIT(reg_ptr->CR1, I2C_CR1_STOP);
			}

			i2c_ptr->rxbuffer[i2c_ptr->i] = READ_REG(reg_ptr->DR);
			i2c_ptr->i++;

			i2c_ptr->rxsize--;
			if(i2c_ptr->rxsize == 0)
			{
				for(int j=0; j<i2c_ptr->rxbuffersize; j++)
				{
					*(i2c_ptr->rx) = i2c_ptr->rxbuffer[j];
					i2c_ptr->rx++;
				}
				__NOP();
			}
		}

	}
}

static void I2C_error(I2C_TypeDef* reg_ptr)
{
	if(READ_BIT(reg_ptr->SR1, I2C_SR1_BERR))
	{
		CLEAR_BIT(reg_ptr->SR1, I2C_SR1_BERR);
		ITM_SendString("BERR\n");
	}
	else if(READ_BIT(reg_ptr->SR1, I2C_SR1_ARLO))
	{
		CLEAR_BIT(reg_ptr->SR1, I2C_SR1_ARLO);
		ITM_SendString("ARLO\n");
	}
	else if(READ_BIT(reg_ptr->SR1, I2C_SR1_AF))
	{
		CLEAR_BIT(reg_ptr->SR1, I2C_SR1_AF);
		ITM_SendString("AF\n");
	}
	else if(READ_BIT(reg_ptr->SR1, I2C_SR1_OVR))
	{
		CLEAR_BIT(reg_ptr->SR1, I2C_SR1_OVR);
		ITM_SendString("OVR\n");
	}
	else if(READ_BIT(reg_ptr->SR1, I2C_SR1_TIMEOUT))
	{
		CLEAR_BIT(reg_ptr->SR1, I2C_SR1_TIMEOUT);
		ITM_SendString("TIMEOUT\n");
	}
}

void I2C1_EV_IRQHandler()
{
	I2C_Interrupt(&i2c1);
}

void I2C2_EV_IRQHandler()
{
	I2C_Interrupt(&i2c2);
}

void I2C3_EV_IRQHandler()
{
	I2C_Interrupt(&i2c3);
}

void I2C1_ER_IRQHandler()
{
	I2C_error(I2C1);
}

void I2C2_ER_IRQHandler()
{
	I2C_error(I2C2);
}

void I2C3_ER_IRQHandler()
{
	I2C_error(I2C3);
}


/*
 * I2C peripheral busy indication
 * Parameter 1: I2C1, I2C2, I2C3
 * returns true if communication is in progress on the bus
 */
bool i2c_busy(I2C_TypeDef* reg_ptr)
{
	if(READ_BIT(reg_ptr->SR2, I2C_SR2_BUSY) || READ_BIT(reg_ptr->CR1, I2C_CR1_START))
		return true;
	else
		return false;
}

/*
 * I2C peripheral initialization
 * Parameter 1: I2C1, I2C2, I2C3
 * Parameter 2: true -> 400kHz	false -> 100kHz
 * Parameter 3: APB1 clock
 */
void i2c_init(I2C_TypeDef* reg_ptr, bool fastMode, uint32_t apb1clk)
{
	if(READ_BIT(reg_ptr->CR1, I2C_CR1_PE))
		return;

	if (reg_ptr==I2C1)
	{
		// Enable Port B
		SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);

		// I2C1 PB8=SCL, alternate function 4 open-drain
		MODIFY_REG(GPIOB->AFR[1], GPIO_AFRH_AFSEL8, 4<<GPIO_AFRH_AFSEL8_Pos);
		MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER8, GPIO_MODER_MODER8_1);
		SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_8);

		// I2C1 PB9=SDA, alternate function 4 open-drain
		MODIFY_REG(GPIOB->AFR[1], GPIO_AFRH_AFSEL9, 4<<GPIO_AFRH_AFSEL9_Pos);
		MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER9, GPIO_MODER_MODER9_1);
		SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_9);

		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);

		NVIC_EnableIRQ(I2C1_EV_IRQn);
		NVIC_EnableIRQ(I2C1_ER_IRQn);
	}

	if (reg_ptr==I2C2)
	{
		// Enable Port B
		SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);

		// I2C2 PB10=SCL, alternate function 4 open-drain
		MODIFY_REG(GPIOB->AFR[1], GPIO_AFRH_AFSEL10, 4<<GPIO_AFRH_AFSEL10_Pos);
		MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER10, GPIO_MODER_MODER10_1);
		SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_10);

		// I2C2 PB11=SDA, alternate function 4 open-drain
		MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL11, 4<<GPIO_AFRH_AFSEL11_Pos);
		MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER11, GPIO_MODER_MODER11_1);
		SET_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_11);

		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);

		NVIC_EnableIRQ(I2C2_EV_IRQn);
		NVIC_EnableIRQ(I2C2_ER_IRQn);
	}

	if (reg_ptr==I2C3)
	{
		SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);
		SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);

		// I2C3 PA8=SCL, alternate function 3 open-drain
		MODIFY_REG(GPIOA->AFR[1], GPIO_AFRH_AFSEL8, 4<<GPIO_AFRH_AFSEL8_Pos);
		MODIFY_REG(GPIOA->MODER, GPIO_MODER_MODER8, GPIO_MODER_MODER8_1);
		SET_BIT(GPIOA->OTYPER, GPIO_OTYPER_OT_8);

		// I2C3 PB4=SDA, alternate function 8 open-drain
		MODIFY_REG(GPIOB->AFR[0], GPIO_AFRL_AFSEL4, 9<<GPIO_AFRL_AFSEL4_Pos);
		MODIFY_REG(GPIOB->MODER, GPIO_MODER_MODER4, GPIO_MODER_MODER4_1);
		SET_BIT(GPIOB->OTYPER, GPIO_OTYPER_OT_4);

		SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C3EN);

		NVIC_EnableIRQ(I2C3_EV_IRQn);
		NVIC_EnableIRQ(I2C3_ER_IRQn);
	}

	// Disable the I2C peripheral
	CLEAR_BIT(reg_ptr->CR1, I2C_CR1_PE);

	// Configure timing
	MODIFY_REG(reg_ptr->CR2, I2C_CR2_FREQ, apb1clk/1000000);

	if (fastMode)
	{
		SET_BIT(reg_ptr->CCR, I2C_CCR_FS);
		MODIFY_REG(reg_ptr->CCR, I2C_CCR_CCR, apb1clk/800000);
		MODIFY_REG(reg_ptr->TRISE, I2C_TRISE_TRISE, apb1clk/4000000+1);
	}
	else
	{
		MODIFY_REG(reg_ptr->CCR, I2C_CCR_CCR, apb1clk/200000);
		MODIFY_REG(reg_ptr->TRISE, I2C_TRISE_TRISE, apb1clk/1000000+1);
	}

	//Enable Interrupts
	SET_BIT(reg_ptr->CR2, I2C_CR2_ITBUFEN);
	SET_BIT(reg_ptr->CR2, I2C_CR2_ITEVTEN);
	SET_BIT(reg_ptr->CR2, I2C_CR2_ITERREN);

	// Enable the peripheral
	SET_BIT(reg_ptr->CR1, I2C_CR1_PE);
}

/*
 * Start I2C communication
 * Parameter 1: I2C1, I2C2, I2C3
 * Parameter 2: 7 bit slave address
 * Parameter 3: pointer to send data buffer (0 if unused)
 * Parameter 4: number of bytes to send
 * Parameter 5: pointer to receive data buffer (0 if unused)
 * Parameter 6:	number of bytes to receive
 */
void i2c_start(I2C_TypeDef* reg_ptr, uint8_t addr, uint8_t* send_buffer, int send_size, uint8_t* receive_buffer, int receive_size)
{
	if(READ_BIT(reg_ptr->SR2, I2C_SR2_BUSY))
		return;

	__disable_irq();

	i2c_s* i2c_ptr;

	if (reg_ptr==I2C1)
		i2c_ptr = &i2c1;
	if (reg_ptr==I2C2)
		i2c_ptr = &i2c2;
	if (reg_ptr==I2C3)
		i2c_ptr = &i2c3;

	i2c_ptr->saddr = addr;
	i2c_ptr->txsize = send_size;
	i2c_ptr->tx = send_buffer;
	i2c_ptr->rxsize = receive_size;
	i2c_ptr->rxbuffersize = receive_size;
	i2c_ptr->rx = receive_buffer;
	i2c_ptr->i = 0;

	__enable_irq();

	SET_BIT(reg_ptr->CR1, I2C_CR1_START);
}
