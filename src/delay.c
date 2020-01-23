#include <stm32f10x.h>
#include "stdint.h"
#include "delay.h"


static __IO uint32_t  	TimingDelay;
static volatile uint64_t  	_micros;

//72 MHz/1000000 = 72,  Every 1 Usec the timer triggers a call to the SysTick_Handler
void delay_init(void) {
	SysTick_Config(72);
	_micros = 0;
}

void SysTick_Handler() {
	if (TimingDelay != 0) TimingDelay--;
	_micros++;
}

void delay_us(uint32_t nTime) {
	TimingDelay = nTime;
	while (TimingDelay != 0);
}

void delay_ms(uint32_t nTime) {
	TimingDelay = nTime*1000;
	while (TimingDelay != 0);
}

uint32_t millis(void){
	return _micros/1000;
}
