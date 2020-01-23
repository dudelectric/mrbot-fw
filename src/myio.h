#ifndef MYIO_H
#define MYIO_H


#define RCC_GREEN		RCC_APB2Periph_GPIOC
#define PIN_GREEN		GPIO_Pin_13
#define PORT_GREEN		GPIOC

void myio_green_led_init();
void myio_green_led_on();
void myio_green_led_off();

#endif
