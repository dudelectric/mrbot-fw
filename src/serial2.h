/*
 * serial2.h
 *
 *  Created on: Sep 8, 2019
 *      Author: Sigit
 */

#ifndef SERIAL2_H_
#define SERIAL2_H_

#define RX_BUFFER_SIZE 250

extern void serial2_begin(uint32_t baud);
extern uint8_t serial2_available();
extern uint8_t serial2_read();
extern void serial2_print(const char *str);
extern void serial2_println(const char *str);
extern void serial2_print_itoa(uint16_t ints);
extern void serial2_putchar(char ch);

#endif /* SERIAL2_H_ */
