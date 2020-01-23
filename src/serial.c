#include <stm32f10x.h>
#include <stdlib.h>
#include "serial.h"



typedef struct _ring_buffer {
	unsigned char buffer[RX_BUFFER_SIZE];
	int head;
	int tail;
} ring_buffer;
ring_buffer rx_buffer[1] =  { { 0 }, 0, 0 };

void store_char(uint8_t c, ring_buffer *rx_buffer){
	uint8_t i = (rx_buffer->head + 1) % RX_BUFFER_SIZE;
	if (i != rx_buffer->tail) {
		rx_buffer->buffer[rx_buffer->head] = c;
		rx_buffer->head = i;
	}
}
void USART1_IRQHandler(void) {
	if ((USART1->SR & USART_SR_RXNE) != 0) {
		uint8_t c = USART1->DR;
		store_char(c, &rx_buffer[0]);
	}
}

void serial_begin(uint32_t baud) {
	/* Enable USART1 and GPIOA clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

	/* NVIC Configuration */
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable the USARTx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Configure the GPIOs */
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART1 Rx (PA.10) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure the USART1 */
	USART_InitTypeDef USART_InitStructure;

	/* USART1 configuration ------------------------------------------------------*/
	/* USART1 configured as follow:
		- BaudRate = 9600 baud
		- Word Length = 8 Bits
		- One Stop Bit
		- No parity
		- Hardware flow control disabled (RTS and CTS signals)
		- Receive and transmit enabled
		- USART Clock disabled
		- USART CPOL: Clock is active low
		- USART CPHA: Data is captured on the middle
		- USART LastBit: The clock pulse of the last data bit is not output to
			the SCLK pin
	 */
	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);

	/* Enable USART1 */
	USART_Cmd(USART1, ENABLE);

	/* Enable the USART1 Receive interrupt: this interrupt is generated when the
		USART1 receive data register is not empty */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

//void serial_begin(uint32_t baud) {
//	RCC->APB2ENR |= (RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN);
//	GPIOA->CRH |= (GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1 | GPIO_CRH_CNF10_0);
//	if (baud == 9600){
//		USART1->BRR = 0x1d4d;
//		USART1->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);
//		USART1->CR1 |= USART_CR1_RXNEIE;
//		NVIC_EnableIRQ (USART1_IRQn);
//	}
//}
uint8_t serial_available(){
	return (RX_BUFFER_SIZE + rx_buffer->head - rx_buffer->tail) % RX_BUFFER_SIZE;
}
uint8_t serial_read(){
     if (rx_buffer->head == rx_buffer->tail) {
           return -1;
     } else {
           uint8_t c =
                 rx_buffer->buffer[rx_buffer->tail];
           rx_buffer->tail =
                 (rx_buffer->tail + 1) % RX_BUFFER_SIZE;
           return c;
     }
}
void serial_print(const char *str) {
	while(*str){
    serial_putchar(*str);
    str++;
  }
}
void serial_println(const char *str) {
  while(*str){
    serial_putchar(*str);
    str++;
  }
  serial_putchar('\r');
  serial_putchar('\n');
}
void serial_print_itoa(uint16_t ints) {
	char cid[6];
	const char* str = itoa(ints,cid,10);
	while(*str){
		serial_putchar(*str);
		str++;
	}
}
void serial_putchar(char ch){
	while (!(USART1->SR & USART_SR_TC));
	USART1->DR = ch;
}



