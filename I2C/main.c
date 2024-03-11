#include <stdint.h>
#include "stm32f4xx.h"

#define GPIOBEN (1U<<1)	// pin 0 = GPIOA and  pin 1 = GPIOB
#define I2C1EN (1U<<21)
#define I2C_100KHZ 80
#define SD_MODE_MAX_RISE_TIME 17
#define CR1_PE (1U<<0)

// Creating symbolic name for necessary Registers
#define SR2_BUSY (1U<<1)

#define SR1_SB (1U<<0)
#define SR1_ADDR (1U<<1)
#define SR1_TXE (1U<<7)
#define SR1_RXNE (1U<<6)
#define SR1_BTF (1U<<2)

#define CR1_START (1U<<8)
#define CR1_PE (1U<<0)
#define CR1_ACK (1U<<10)
#define CR1_STOP (1U<<9)

// for adxl
#define DEVID_R (0X00)
#define DATA_FORMAT_R (0X31)
#define POWER_CTL_R (0X2D)
#define DATA_START_ADDR (0X32)
#define DEVICE_ADDR (0X53)

#define FOUR_G (0X01)
#define RESET (0X00)
#define SET_MEASURE_B (0X08)

// PB8 - SCL
// PB9 - SDA

void I2C1_init(void);
void I2C1_ByteRead(char saddr, char maddr, char *data);
void I2C1_BurstRead(char saddr, char maddr, int n, char *data);
void I2C1_BurstWrite(char saddr, char maddr, int n, char *data);
void adxl_init(void);
void adxl_read_values(uint8_t);
void adxl_write(uint8_t, char);
void adxl_read_address(uint8_t);

char data;
uint8_t data_rec[6];

int16_t x, y, z; // we want to read 3 values
float xg, yg, zg;

int main(void){
	adxl_init();
	while(1){
		adxl_read_values(DATA_START_ADDR);
		x = ((data_rec[1]<<8) | data_rec[0]);
		y = ((data_rec[3]<<8) | data_rec[2]);
		z = ((data_rec[5]<<8) | data_rec[4]);

		xg = (x*0.0078);	// 0.0078 from data sheet
		yg = (y*0.0078);
		zg = (z*0.0078);
	}
}

void adxl_init(void){
	I2C1_init();
	adxl_read_address(DEVID_R);
	adxl_write(DATA_FORMAT_R, FOUR_G);

	// Reset all bits
	adxl_write(POWER_CTL_R, RESET);

	// Measure the Bits
	adxl_write(POWER_CTL_R, SET_MEASURE_B);
}

void adxl_read_values(uint8_t reg){
	I2C1_BurstRead(DEVICE_ADDR, reg, 6, (char*)data_rec);
}

void adxl_write(uint8_t reg, char value){
	char data[1];
	data[0] = value;
	I2C1_BurstWrite(DEVICE_ADDR, reg, 1, &data);
}

void adxl_read_address(uint8_t reg){
	I2C1_ByteRead(DEVICE_ADDR, reg, &data);
}

void I2C1_init(void){
	// Enabling CLK access to PORT B
	RCC->AHB1ENR |= GPIOBEN;

	// Setting PB8 and PB9 to AF
	GPIOB->MODER &= ~(1U<<16);
	GPIOB->MODER |= (1U<<17);

	GPIOB->MODER &= ~(1U<<18);
	GPIOB->MODER |= (1U<<19);

	// Setting output type as OPEN DRAIN
	GPIOB->OTYPER |= (1U<<8);
	GPIOB->OTYPER |= (1U<<9);

	// Set PULL-UP for PB8 and PB9
	GPIOB->PUPDR |= (1U<<16);
	GPIOB->PUPDR &= ~(1U<<17);

	GPIOB->PUPDR |= (1U<<18);
	GPIOB->PUPDR &= ~(1U<<19);

	// Set the AF type for I2C
	GPIOB->AFR[1] &= ~(1U<<0);
	GPIOB->AFR[1] &= ~(1U<<1);
	GPIOB->AFR[1] |= (1U<<2);
	GPIOB->AFR[1] &= ~(1U<<3);

	GPIOB->AFR[1] &= ~(1U<<4);
	GPIOB->AFR[1] &= ~(1U<<5);
	GPIOB->AFR[1] |= (1U<<6);
	GPIOB->AFR[1] &= ~(1U<<7);

	// Test - Enabling I2C1, therefore enabling APB1
	RCC->APB1ENR |= I2C1EN;

	// Reset the I2C
	I2C1->CR1 |= (1U<<15);

	// Come out of Reset Mode
	I2C1->CR1 &= ~(1U<<15);

	// Setting the peripheral clock freq
	I2C1->CR2 = (1U<<4);

	I2C1->CCR = I2C_100KHZ;

	I2C1->TRISE = SD_MODE_MAX_RISE_TIME;

	I2C1->CR1 |= CR1_PE;
}

void I2C1_ByteRead(char saddr, char maddr, char *data){
	// saddr = slave address
	// maddr = memory address
	// data = data pointer

	volatile int temp;
	// STEPS
	// wait until bus is not busy
	while(I2C1->SR2 & (SR2_BUSY)){

	}

	// Once available for transfer, Generate Start Condition
	I2C1->CR1 |= CR1_START;

	// Wait until the start flag is set
	while(!(I2C1->SR1 & (SR1_SB))){

	}

	// Transmit the slave address + write
	I2C1->DR = saddr<<1;

	// Wait until the address flag is set
	while(!(I2C1->SR1 & (SR1_ADDR))){

	}

	// Clear Address Flag
	temp = I2C1->SR2;

	//Send memory address
	I2C1->DR = maddr;

	// Wait until the Transmitter is empty
	while(!(I2C1->SR1 & (SR1_TXE))){

	}

	//Generatr resart
	I2C1->CR1 |=CR1_START;

	// wait until start flag is set
	while(!(I2C1->SR1 & (SR1_SB))){

	}

	// Transmit slave address + read
	I2C1->DR = saddr<<1 | 1;

	// Wait until the address Flag is set
	while(!(I2C1->SR1 & (SR1_ADDR))){

	}

	// Once set, disable the Acknowledgment
	I2C1->CR1 &= ~CR1_ACK;

	// Clear the Address Flag
	temp = I2C1->SR2;

	// Generate Stop after data receive
	I2C1->CR1 |= CR1_STOP;

	// Wait until Receiver enable flag is set
	while(!(I2C1->SR1 & (SR1_RXNE))){

	}

	// Read data from Data Register
	(*data++) = I2C1->DR;
}

void I2C1_BurstRead(char saddr, char maddr, int n, char *data){
	volatile int temp;
	// STEPS
	// wait until bus is not busy
	while(I2C1->SR2 & (SR2_BUSY)){

	}

	// Once available for transfer, Generate Start Condition
	I2C1->CR1 |= CR1_START;

	// Wait until the start flag is set
	while(!(I2C1->SR1 & (SR1_SB))){

	}

	// Transmit the slave address + write
	I2C1->DR = saddr<<1;

	// Wait until the address flag is set
	while(!(I2C1->SR1 & (SR1_ADDR))){

	}

	// Clear Address Flag
	temp = I2C1->SR2;

	// Wait until the Transmitter is empty
	while(!(I2C1->SR1 & (SR1_TXE))){

	}

	// Send Memory Address
	I2C1->DR = maddr;

	// Wait until the Transmitter is empty
	while(!(I2C1->SR1 & (SR1_TXE))){

	}

	// Once Empty, generate Restart
	I2C1->CR1 |= CR1_START;

	// wait until start flag is set
	while(!(I2C1->SR1 & (SR1_SB))){

	}

	// Transmit slave address + read
	I2C1->DR = saddr<<1 | 1;

	// Wait until the address Flag is set
	while(!(I2C1->SR1 & (SR1_ADDR))){

	}

	// Clear the Address Flag
	temp = I2C1->SR2;

	// Enable Acknowledgment
	I2C1->CR1 |= CR1_ACK;
	while(n>0U){
		if(n==1U){
			I2C1->CR1 &= ~CR1_ACK;	// Disable ACK
			I2C1->CR1 |= CR1_STOP;	// Generate Stop

			// Wait until Receiver enable flag is set
			while(!(I2C1->SR1 & (SR1_RXNE))){

			}

			// Read data from Data Register
			(*data++) = I2C1->DR;

			break;
		}else{
			// Wait until Receiver enable flag is set
			while(!(I2C1->SR1 & (SR1_RXNE))){

			}

			// Read data from Data Register
			(*data++) = I2C1->DR;
			n--;
		}
	}
}

void I2C1_BurstWrite(char saddr, char maddr, int n, char *data){
	volatile int temp;
	// STEPS
	// wait until bus is not busy
	while(I2C1->SR2 & (SR2_BUSY)){

	}

	// Once available for transfer, Generate Start Condition
	I2C1->CR1 |= CR1_START;

	// Wait until the start flag is set
	while(!(I2C1->SR1 & (SR1_SB))){

	}

	// Transmit the slave address + write
	I2C1->DR = saddr<<1;

	// Wait until the address flag is set
	while(!(I2C1->SR1 & (SR1_ADDR))){

	}

	// Clear Address Flag
	temp = I2C1->SR2;

	// Wait until the Transmitter is empty
	while(!(I2C1->SR1 & (SR1_TXE))){

	}

	// Send Memory Address
	I2C1->DR = maddr;

	for(int i=0; i<n; i++){
		// Wait until the Transmitter is empty
		while(!(I2C1->SR1 & (SR1_TXE))){

		}
		I2C1->DR = (*data++);
	}

	while(!(I2C1->SR1 & (SR1_BTF))){

	}
	// Generate Stop after data receive
	I2C1->CR1 |= CR1_STOP;
}
