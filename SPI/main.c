#include <stdint.h>
#include "stm32f4xx.h"

#define GPIOAEN (1U<<0)
#define SPI1EN (1U<<12)

#define SR_TXE (1U<<1)
#define SR_BSY (1U<<7)
#define SR_RXNE (1U<<0)

#define MULTI_BYTE_EN (0X40)
#define READ_OPERATION (0X80)

// MISO -> PA6
// MOSI -> PA7
// CLK -> PA5
// CS -> PA9
// AF05

// for adxl
#define DEVID_R (0X00)
#define DATA_FORMAT_R (0X31)
#define POWER_CTL_R (0X2D)
#define DATA_START_ADDR (0X32)
#define DEVICE_ADDR (0X53)
#define FOUR_G (0X01)
#define RESET (0X00)
#define SET_MEASURE_B (0X08)

void adxl_init(void);
void adxl_read(uint8_t address, uint8_t *RxData);
void adxl_write(uint8_t address, char value);

void spi_config(void);
void spi_gpio_init(void);
void spi_transmit(uint8_t *data, uint32_t size);
void spi_recieve(uint8_t *data, uint32_t size);
void cs_disable(void);
void cs_enable(void);

char data;
uint8_t data_rec[6];

int16_t x, y, z; // we want to read 3 values
float xg, yg, zg;

int main(void){
	adxl_init();

	// Forever Loop
	while(1){
		adxl_read(DATA_START_ADDR, data_rec);
		x = ((data_rec[1]<<8) | data_rec[0]);
		y = ((data_rec[3]<<8) | data_rec[2]);
		z = ((data_rec[5]<<8) | data_rec[4]);

		xg = (x*0.0078);	// 0.0078 from data sheet
		yg = (y*0.0078);
		zg = (z*0.0078);
	}
}

void adxl_init(void){
	spi_gpio_init();
	spi_config();

	// Set Data Format Range
	adxl_write(DATA_FORMAT_R, FOUR_G);

	// Reset all bits
	adxl_write(POWER_CTL_R, RESET);

	// Configure Power Control Measure Bits
	adxl_write(POWER_CTL_R, SET_MEASURE_B);
}

void adxl_read(uint8_t address, uint8_t *RxData){
	// Set Read Operation
	address |= READ_OPERATION;

	// Enable Multibyte
	address |= MULTI_BYTE_EN;

	// Pull CS Line to low, to enable Slave
	cs_enable();

	// Transmit Data and Address
	spi_transmit(&address, 1);

	spi_recieve(RxData, 6);	// x, y, z with 2 values each, therefore 6

	// Pull CS Line to High, to disable Slave
	cs_disable();
}

void adxl_write(uint8_t address, char value){
	uint8_t data[2];

	// Enable Multibyte and Plave Address into Buffer
	data[0] = address | MULTI_BYTE_EN;
	data[1] = value;

	// Pull CS Line Low to enable Slave
	cs_enable();

	// Transmit Data and Address
	spi_transmit(data, 2);

	// Pull CS Line High to disable Slave
	cs_disable();
}

void spi_gpio_init(void){
	RCC->AHB1ENR = GPIOAEN;		// Enabling the CLK for AHB1

	// Setting MODER for PA5 to AF
	GPIOA->MODER &= ~(1U<<10);
	GPIOA->MODER |= (1U<<11);

	// Setting MODER for PA6 to AF
	GPIOA->MODER &= ~(1U<<12);
	GPIOA->MODER |= (1U<<13);

	// Setting MODER for PA7 to AF
	GPIOA->MODER &= ~(1U<<14);
	GPIOA->MODER |= (1U<<15);

	// Setting MODER for PA9 to Output, as it is for Chip Select
	GPIOA->MODER |= (1U<<18);
	GPIOA->MODER &= ~(1U<<19);

	// Setting AF Registers for PA5
	GPIOA->AFR[0] |= (1U<<20);
	GPIOA->AFR[0] &= ~(1U<<21);
	GPIOA->AFR[0] |= (1U<<22);
	GPIOA->AFR[0] &= ~(1U<<23);

	// Setting AF Registers for PA6
	GPIOA->AFR[0] |= (1U<<24);
	GPIOA->AFR[0] &= ~(1U<<25);
	GPIOA->AFR[0] |= (1U<<26);
	GPIOA->AFR[0] &= ~(1U<<27);

	// Setting AF Registers for PA7
	GPIOA->AFR[0] |= (1U<<28);
	GPIOA->AFR[0] &= ~(1U<<29);
	GPIOA->AFR[0] |= (1U<<30);
	GPIOA->AFR[0] &= ~(1U<<31);
}

void spi_config(void){
	// Enabled CLK for SPI in APB2
	RCC->APB2ENR |= SPI1EN;

	// Set Clock for SPI
	// FPCLK/4 - 001
	SPI1->CR1 |= (1U<<3);
	SPI1->CR1 &= ~(1U<<4);
	SPI1->CR1 &= ~(1U<<5);

	// Setting CPOL and CPHA to 1
	SPI1->CR1 |= (1U<<0);
	SPI1->CR1 |= (1U<<1);

	// Enable Full Duplex
	SPI1->CR1 &= ~(1U<<10);

	// Set MSB transmit to First
	SPI1->CR1 &= ~(1U<<7);

	// Setting Master
	SPI1->CR1 |= (1U<<2);

	// set 8 bit data mode
	SPI1->CR1 &= ~(1U<<11);

	// setting Software Slave Management and SSI
	SPI1->CR1 |= (1U<<8);
	SPI1->CR1 |= (1U<<9);

	// SPI Enable
	SPI1->CR1 |= (1U<<6);
}

void spi_transmit(uint8_t *data, uint32_t size){
	uint32_t i = 0;
	uint8_t temp;

	while(i<size){

		// we wait until transmit buffer is not empty, that is until TXE is set
		while(!(SPI1->SR & SR_TXE)){}

		// write data to the data register
		SPI1->DR = data[i];
		i++;
	}
	// we wait until transmit buffer is not empty, that is until TXE is set
	while(!(SPI1->SR & SR_TXE)){}

	// we wait until transmit buffer is not empty, that is until TXE is set
	while(!(SPI1->SR & SR_BSY)){}

	// to clear, we need to just read the value, twice, to clear the OVERFLOW Flag
	temp = SPI1->DR;
	temp = SPI1->DR;
}

void spi_recieve(uint8_t *data, uint32_t size){
	while(size){
		SPI1->DR = 0;	// sending dummy data

		while(!(SPI1->SR & SR_RXNE)){}	// we wait until Receiver buffer is empty

		(*data++) = SPI1->DR;
		size--;
	}
}

void cs_enable(void){
	GPIOA->ODR &= ~(1U<<9);
	// since we selected PA9 as Output for Chip Select
	// therefore we enable it and disable it
}

void cs_disable(void){
	GPIOA->ODR |= (1U<<9);
}
