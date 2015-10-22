#ifndef MUART_H
#define MUART_H

	#ifndef F_CPU
	#define F_CPU 16000000UL // zegar w Hz
	#error F_CPU not defined for muart.h! Using default 16000000UL
	#endif
	
	#define RS_BAUD 9600
	#define RS_UBRR ((F_CPU / 16) / RS_BAUD) - 1
	
	void uart_init(void);
	void uart_putc(uint8_t data);
	void uart_puts(const char *s );
	uint8_t uart_ischar(void);
	uint8_t uart_getc(void);	

#endif
