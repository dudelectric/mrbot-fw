#ifndef SERIAL_H
#define SERIAL_H

#define RX_BUFFER_SIZE 250

extern void serial_begin(uint32_t baud);
extern uint8_t serial_available();
extern uint8_t serial_read();
extern void serial_print(const char *str);
extern void serial_println(const char *str);
extern void serial_print_itoa(uint16_t ints);
extern void serial_putchar(char ch);

#endif
