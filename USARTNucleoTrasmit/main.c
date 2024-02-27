#include <stdint.h>

#define GPIOAEN (1U<<0)		//  as GPIOA is 1st pin of RCC ENR
#define UART2EN (1U<<17)    // 17th pin of ABP1 is USART2

#define SYS_FREQ 16000000
#define APB1_CLK SYS_FREQ
#define UART_BAUDRATE 115200

void uart2_tx_init(void);
static uint16_t compute_uart_bd(uint32_t , uint32_t);   // a function to calculate Baud Rate
void uart2_write(int);

// pointing to all the register
uint32_t *AHB1ENR = (uint32_t*) 0X40023830;
uint32_t *APB1ENR = (uint32_t*) 0X40023840;
uint32_t *GPIOA_MODER = (uint32_t*) 0X40020000;
uint32_t *GPIO_AFRL = (uint32_t*) 0X40020020;
uint32_t *USART2_SR = (uint32_t*) 0X40004400;
uint32_t *USART2_DR = (uint32_t*) 0X40004404;
uint32_t *USART2_BRR = (uint32_t*) 0X40004408;	// Baud Rate Register
uint32_t *USART2_CR1 = (uint32_t*) 0X4000440C;

int main(void){
	uart2_tx_init();
	while(1){
		uart2_write('A');
		// This data can be viewed using any serial Monitor
		// in Arduino select COM Port and then BaudRate in Serial Monitor
		// in RealTerm Terminal, do the same and then hit OPEN
	}
}

void uart2_tx_init(void){
	// Enabling CLK access for GPIOA
	*AHB1ENR |= GPIOAEN;

	// setting GPIOA MODER
	*GPIOA_MODER &= ~(1U<<4);	// 4th pin will be made 0 for sure
	*GPIOA_MODER |= (1U<<5);	// 5th pin will be made 1 for sure

	// setting GPIO Port2 AFRL mode to AF7 (Alternate Function)
	*GPIO_AFRL |= (1U<<8);
	*GPIO_AFRL |= (1U<<9);
	*GPIO_AFRL |= (1U<<10);
	*GPIO_AFRL &= ~(1U<<11);

	// USART Configure
	// Enabling UART2EN, it is in APB1
	*APB1ENR |= UART2EN;

	// putting BRR Value
	*USART2_BRR = compute_uart_bd(APB1_CLK, UART_BAUDRATE);

	// configure the Transmit Direction
	*USART2_CR1 |= (0X2008);
//	*USART2_CR1 = 0X0008;
//	*USART2_CR1 |= 0X2000;

}

// to calculate Baud Rate
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t Baudrate){
	return ( (PeriphClk + (Baudrate/2U))/Baudrate );
}

void uart2_write(int ch){
	// Make sure that the Transmit Data Register TDR is Empty, SR contains TDR
	while(!(*USART2_SR & 0X0080)){
		// It will keep on looping here, till TDR is turned ON, that is data is send
	}
	// When TDR = 1, then we can write to the Data Register
	*USART2_DR = (ch & 0XFF);	// ch is for character
}
