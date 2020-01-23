#include <stm32f10x.h>
#include "myio.h"

//#define RGB_COMMON_CATHODE
#define RGB_COMMON_ANODE

//green
void myio_green_led_init() {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_GREEN, ENABLE);
	GPIO_InitStructure.GPIO_Pin = PIN_GREEN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(PORT_GREEN, &GPIO_InitStructure);
}
void  myio_green_led_on(){
#ifdef  RGB_COMMON_CATHODE
	GPIO_WriteBit(PORT_GREEN, PIN_GREEN, SET);
#endif
#ifdef  RGB_COMMON_ANODE
	GPIO_WriteBit(PORT_GREEN, PIN_GREEN, RESET);
#endif
}

void  myio_green_led_off(){
#ifdef  RGB_COMMON_CATHODE
	GPIO_WriteBit(PORT_GREEN, PIN_GREEN, RESET);
#endif
#ifdef  RGB_COMMON_ANODE
	GPIO_WriteBit(PORT_GREEN, PIN_GREEN, SET);
#endif
}



