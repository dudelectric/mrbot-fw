#ifndef DELAY_H
#define DELAY_H

extern void delay_init(void);
extern void delay_us(uint32_t nTime);
extern void delay_ms(uint32_t nTime);
extern uint32_t millis(void);

#endif
