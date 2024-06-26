#ifndef UART_H
#define UART_H

char* int_to_string(int number);
// this function should be called before the main loop to set up the UART registers
void setup_UART(void);

// Transmitting functions
void UART_transmit_char(char data);
void UART_transmit_string(char *string);
void UART_transmit_int(int number);

// Receiving functions
char UART_getc(void);
int UART_get_int(void);

#endif