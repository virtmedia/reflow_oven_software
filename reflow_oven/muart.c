#include <avr/io.h>
#include <inttypes.h> 
#include "muart.h"

void uart_init(void)
{
#if (defined (__AVR_ATmega8__)) || (defined (__AVR_ATmega8A__))
	// Ustawienie prêdkoœci transmisji
	//UBRRH &= ~(1 << URSEL);
	//UBRRH = (uint8_t)(RS_UBRR >> 8);
	UBRRL = (uint8_t)RS_UBRR;

	// W³¹czenie nadajnika i odbiornika
	UCSRB = (1 << RXEN) | (1 << TXEN);
 
	// Ustawienie formatu ramki:
	// 8 bitów danych, 1 bit stopu, brak parzystoœci
	UCSRC =(1<<URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
#else
	// Ustawienie prêdkoœci transmisji
	UBRR0H = (uint8_t)((RS_UBRR) >> 8);
	UBRR0L = (uint8_t)RS_UBRR;
 
	// W³¹czenie nadajnika i odbiornika
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
 
	// Ustawienie formatu ramki:
	// 8 bitów danych, 1 bit stopu, brak parzystoœci
	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
#endif	
}

void uart_putc(uint8_t data)
{
#if (defined (__AVR_ATmega8__)) || (defined (__AVR_ATmega8A__))
	// Oczekiwanie na zakoñczenie nadawania
	while (!(UCSRA & (1 << UDRE)));
 
	// Wys³anie danych
	UDR = data;
#else
	// Oczekiwanie na zakoñczenie nadawania
	while (!(UCSR0A & (1 << UDRE0)));
 
	// Wys³anie danych
	UDR0 = data;
#endif
}
 
// przydatna (ale nie niezbêdna) funkcja wysy³aj¹ca ci¹g znaków
void uart_puts(const char *s )
{
	while (*s) 
		uart_putc(*s++);
}

uint8_t uart_ischar(void)
{
	// Czy w buforze s¹ dane?
#if (defined (__AVR_ATmega8__)) || (defined (__AVR_ATmega8A__))
	return (UCSRA & (1 << RXC));
#else
	return (UCSR0A & (1 << RXC0));
#endif
}
 
uint8_t uart_getc(void)
{
	// Czy w buforze s¹ dane?
	while(!uart_ischar());
 
	// Dane z bufora
#if (defined (__AVR_ATmega8__)) || (defined (__AVR_ATmega8A__))
	return UDR;
#else
	return UDR0;
#endif
}