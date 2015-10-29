/*
 * reflow_oven.c
 *
 * Created: 2014-01-04 11:32:57
 *  Author: Olek

Pinout Description:
PC0 - OUT - LCD D7
PC1 - OUT - LCD D6
PC2 - OUT - LCD D5
PC3 - OUT - LCD D4
PC4 - OUT - N/U
PC5 - OUT - N/U

PD0 - OUT - TRIAK OUT
PD1 - OUT - N/U
PD2 - IN  - BUTTON (INT0)
PD3 - IN  - Encoder_A (INT1)
PD4 - IN  - Encoder_B (PCINT20)
PD5 - OUT - Backlight PWM (OC0B)
PD6 - OUT - Contrast PWM (OC0A)
PD7 - OUT - LCD RS

PB0 - OUT - LCD E
PB1 - OUT - Speaker PWM (OC1A)
PB2 - OUT - MAX6675 /CS
PB3 - OUT - MOSI N/U
PB4 - IN  - MISO
PB5 - OUT - CLK
PB6 - OUT - N/U
PB7 - OUT - N/U

 
 */ 

//F_CPU=8 000 000 UL
#include <avr/io.h>
#include <inttypes.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/fuse.h>
#include <avr/sleep.h>
#include <stdlib.h>	//If use atoi();
//#include "muart.h"
#include "HD44780.h"

#define TCCR1B_ON	_BV(WGM12)|_BV(CS10)
#define TCCR1B_OFF	_BV(WGM12)

typedef struct
{
	uint32_t powerup;
	uint8_t backlight; //record 1st place
	uint8_t contrast; //record 2nd place
	uint8_t t_alpha;
	int8_t t_offset;
	volatile uint8_t setTemperature;
} eemem_t;

void initSequence(void);
void eepromInit(void);
void eepromSave(void);
void ioInit(void);
void timerInit(void);
void pinInterruptsInit(void);
void setBacklight(uint8_t brightnes);
void setContrast(uint8_t contrast);
void speakerOn(void);
void speakerOff(void);
void speakerBeep(uint16_t duration); //duration in ms
void lcd8t(uint8_t l);	//print on lcd decimal number
void lcd16t(uint16_t l);
void lcdInt(uint16_t number);
void adcInit(void);
uint8_t readADCTemp(void);
void spiInit(void);
uint8_t spiByte(uint8_t data);
uint16_t max6675Read(void);
uint16_t max6675GetTemperature(uint16_t max6675Data);
void LCD_WriteTemperature(uint16_t max6675Data);
inline void heaterOn(void);
inline void heaterOff(void);

//Global variables:
eemem_t EEMEM eemem = {.powerup = 0, .backlight = 60, .contrast = 50, .t_alpha = 10, .t_offset = 0, .setTemperature=30};
eemem_t mem;

volatile uint8_t encoderPosition = 0;
volatile uint8_t adcTemp;
volatile uint8_t state=1;
volatile uint16_t max6675Data;

int main(void)
{
	initSequence();
	
	LCD_Clear();
	sei();
	LCD_GoTo(12,0);
	LCD_WriteData(126);
    while(1)
    {
		if (max6675GetTemperature(max6675Read()) < mem.setTemperature)
		{
			heaterOn();
		}
		else
		{
			heaterOff();
		}
		LCD_GoTo(0,0);
		LCD_WriteTemperature(max6675Read());
		LCD_GoTo(0,1);
		lcdInt(readADCTemp());
		LCD_GoTo(13,0);
		lcd8t(mem.backlight);
		LCD_GoTo(13,1);
		lcd8t(mem.setTemperature);
		
		_delay_ms(200);
        
    }
}




void initSequence(void)
{
	_delay_ms(1000);
	eepromInit();
	ioInit();
	timerInit();
	adcInit();
	readADCTemp();
	spiInit();
	setBacklight(mem.backlight);
	setContrast(mem.contrast);
	pinInterruptsInit();
	//uart_init();	//9600 bod, 1bit stop, parity: none;
	speakerBeep(200);
	LCD_Init();
	LCD_Clear();
	LCD_WriteText("  Reflow oven");
	LCD_GoTo(0,1);
	LCD_WriteText("E.VT0.PL 2015 v0");
	_delay_ms(3000);
	LCD_Clear();
	LCD_WriteText("Pow: Bcl: Con:");
	LCD_GoTo(0,1);
	lcdInt(mem.powerup);
	LCD_GoTo(5,1);
	lcdInt(mem.backlight);
	LCD_GoTo(10,1);
	lcdInt(mem.contrast);
	_delay_ms(3000);
	LCD_Clear();	
}

void eepromInit(void)
{
	eeprom_read_block(&mem, &eemem, sizeof(eemem_t));
	mem.powerup++;
	eepromSave();
	encoderPosition = mem.setTemperature;
}

void eepromSave(void)
{
	eeprom_update_block(&mem,&eemem,sizeof(eemem_t));
}

void ioInit(void)
{
	DDRC = 0xFF;
	DDRD = _BV(PD0)|_BV(PD1)|_BV(PD5)|_BV(PD6)|_BV(PD7);
	DDRB = _BV(PB0)|_BV(PB1)|_BV(PB2)|_BV(PB3)|_BV(PB5)|_BV(PB6)|_BV(PB7);
	
	PORTC = 0;
	PORTD = 0;
	PORTB = _BV(PB2);
}

void timerInit(void)
{
	//Timer 0 Fast PWM for backlight & contrast
	TCCR0A = _BV(COM0A1)|_BV(COM0B1)|_BV(WGM01)|_BV(WGM00);
	TCCR0B = _BV(CS00);
	OCR0A = 0;
	OCR0B = 0;
	//Timer 1 for speaker OC1A
	//CTC, WGM12 = 1, may be eventually WGM13 = 1
	TCCR1A = _BV(COM1A0);
	TCCR1B = TCCR1B_OFF;
	//set timer1 frequency = 2KHz
	OCR1A = 4000;
	
	
}

void pinInterruptsInit(void)
{
	//INT0 on falling edge, INT1 on any edge
	EICRA = _BV(ISC11)|_BV(ISC01);
	EIMSK = _BV(INT1)|_BV(INT0);
}

void setBacklight(uint8_t brightnes)
{
	if (brightnes)
	{
		TCCR0A = _BV(COM0A1)|_BV(COM0B1)|_BV(WGM01)|_BV(WGM00);
	} 
	else
	{
		TCCR0A = 0;
	}
	OCR0B = brightnes;
	mem.backlight = brightnes;
}
void setContrast(uint8_t contrast)
{
	OCR0A = contrast;
	mem.contrast = contrast;
}

void speakerOn(void)
{
	TCCR1B = TCCR1B_ON;
}

void speakerOff(void)
{
	TCCR1B = TCCR1B_OFF;
}

void speakerBeep(uint16_t duration) //duration in ms
{
	speakerOn();
	while(duration--)
		_delay_ms(1);
	speakerOff();
}

void lcd8t(uint8_t l)	//print on lcd decimal number up to 255
{
	uint8_t j,d,s;
	j = l%10;
	l /= 10;
	d = l%10;
	l /= 10;
	s = l%10;
	LCD_WriteData('0'+s);
	LCD_WriteData('0'+d);
	LCD_WriteData('0'+j);
	
}

void lcd16t(uint16_t l)	//print on lcd decimal number upp to 65535
{
	uint8_t j,d,s,t,dt;
	j = l%10;
	l /= 10;
	d = l%10;
	l /= 10;
	s = l%10;
	l /= 10;
	t = l%10;
	l /= 10;
	dt = l%10;
	LCD_WriteData('0'+dt);
	LCD_WriteData('0'+t);
	LCD_WriteData('0'+s);
	LCD_WriteData('0'+d);
	LCD_WriteData('0'+j);
	
}

void lcdInt(uint16_t number)
{
	char str[6];
	itoa(number,str,10);
	if (number < 10)
	{
		LCD_WriteData('0');
	}
	LCD_WriteText(str);
}

void adcInit(void)
{
	//Internal 1.1 reference voltage, temperature sensor;
	ADMUX = _BV(REFS1)|_BV(REFS0)|_BV(MUX3);
	ADCSRB = 0;
	ADCSRA = _BV(ADEN)|_BV(ADSC)|_BV(ADATE)|_BV(ADPS2)|_BV(ADPS1);
	//Wait for conversion to complete
	while(!(ADCSRA & (1<<ADIF)));	
}

uint8_t readADCTemp(void)
{
	 return (uint8_t)(((uint16_t)ADC*(uint16_t)mem.t_alpha)/10)+mem.t_offset;
}


void spiInit(void)
{
	//SPI mode 1; CPOL = 0; CPHA=1; clk/4
	SPCR = _BV(SPE)|_BV(MSTR)|_BV(CPHA);
}

uint8_t spiByte(uint8_t data)
{
	// Start transmission (MOSI)
	SPDR = data;
	// Wait for transmission complete
	while(!(SPSR & (1<<SPIF)));
	// Get return Value;
	return SPDR;
}

uint16_t max6675Read(void)
{
	uint16_t max6675Data;
	PORTB &= ~_BV(PB2); //CS LOW
	_delay_us(1);
	max6675Data = (spiByte(0) << 8);
	max6675Data |= (spiByte(0) & 0xFF);
	_delay_us(1);
	PORTB |= _BV(PB2); //CS HIGH
	return max6675Data;
}

//Return temperature in Celcius 
uint16_t max6675GetTemperature(uint16_t max6675Data)
{
	return (((max6675Data >> 3) & 0xFFF)/4);
}

void LCD_WriteTemperature(uint16_t max6675Data)
{
	if(max6675Data & _BV(2))
	{
		LCD_WriteText("NO PROBE ");
	}
	else
	{	
		lcd8t((((max6675Data >> 3) & 0xFFF)/4));
		LCD_WriteData('.');
		lcdInt( ((max6675Data >> 3) & 0x3)*25 );
		LCD_WriteData(223);
		LCD_WriteData('C');
	}
}

inline void heaterOn(void)
{
	PORTD |= _BV(PD0);
}

inline void heaterOff(void)
{
	PORTD &= ~_BV(PD0);
}

// INTERRUPTS

ISR(INT0_vect)
{
	state = (state+1) % 2;
	if (state)
	{
		encoderPosition = mem.backlight;
		LCD_GoTo(12,0);
		LCD_WriteData(126);
		LCD_GoTo(12,1);
		LCD_WriteData(32);
	} 
	else
	{
		encoderPosition = mem.setTemperature;
		LCD_GoTo(12,0);
		LCD_WriteData(32);
		LCD_GoTo(12,1);
		LCD_WriteData(126);
	}
	eepromSave();
	speakerBeep(20);
}

ISR(INT1_vect)
{
	if (PIND & _BV(PD4))
	{
		if (encoderPosition>0)
		encoderPosition--;
	}
	else
	{
		if (encoderPosition<255)
		encoderPosition++;
	}
	//
	if(state)
	{
		setBacklight(encoderPosition);
	}
	else
	{
		mem.setTemperature=encoderPosition;
		
		
	}
	
}


