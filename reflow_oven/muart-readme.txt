Biblioteka "muart"
Opis: Biblioteka s³u¿y do obs³ugi sprzêtowego UART w procesorach ATMEGA8 oraz ATMEGA48/88/168/328.

U¿ycie: 

1. Skopiuj pliki "muart.h" oraz "muart.c" do katalogu projektu.
2. Wyedytuj plik Makefile (jeœli jest taka potrzeba).
3. W nag³ówkach projektu dodaj #include "muart.h"
4. W muart.h zdefiniuj odpowiednio RS_BAUD (linia 9)
5. U¿yj funkcji inicjalizacyjnej : uart_init();
6. U¿ywaj funkcji z biblioteki:

void uart_init(void);
/*
Inicjalizuje UART. U¿yj jednorazowo przed korzystaniem z transmisji.
*/

void uart_putc(uint8_t data);
/*
Wysy³a pojedyñczy znak 1-bajtowy
*/

void uart_puts(const char *s );
/*
Wysy³a ci¹g znaków, na przyk³ad napis:
uart_puts("NAPIS");
*/

uint8_t uart_ischar(void);
/*
Sprawdza, czy w buforze s¹ odebrane dane; funkcja wewnêtrzna

*/

uint8_t uart_getc(void);
/*
Odbiera znak 1-bajtowy
*/