
#ifndef HD44780_H
#define HD44780_H 120
#include "main.h"
/**
 *	It also supports all HD44780 compatible LCD drivers.
LCD		STM32FXXX		DESCRIPTION

GND		GND				Ground
VCC		+5V				Power supply for LCD
V0		Potentiometer	Contrast voltage. Connect to potentiometer
RS		PB2				Register select, can be overwritten in your project's defines.h file
RW		GND				Read/write
E		PB7				Enable pin, can be overwritten in your project's defines.h file
D0		-				Data 0 - doesn't care
D1		-				Data 1 - doesn't care
D2		-				Data 2 - doesn't care
D3		-				Data 3 - doesn't  care
D4		PC12			Data 4, can be overwritten in your project's defines.h file
D5		PC13			Data 5, can be overwritten in your project's defines.h file
D6		PB12			Data 6, can be overwritten in your project's defines.h file
D7		PB13			Data 7, can be overwritten in your project's defines.h file
A		+3V3			Back light positive power
K		GND				Ground for back light
 *	
 * If you want to change pinout, do this in your defines.h file with lines below and set your own settings:
 *	
*/




#define OUTPUT_MODE					1
#define INPUT_MODE					0

#define RIGHT						32
#define MAIN						16
#define LEFT						0



void HD44780_Init(uint8_t cols, uint8_t rows);
void HD44780_DisplayOn(void);
void HD44780_DisplayOff(void);
void HD44780_Clear(void);
void HD44780_CursorSet(uint8_t col, uint8_t row);
void HD44780_Putc(uint8_t c);
void HD44780_Puts(uint8_t x, uint8_t y, char* str);
void HD44780_Putsn(uint8_t x, uint8_t y, char* str, uint8_t len);
void HD44780_BlinkOn(void);
void HD44780_BlinkOff(void);
void HD44780_CursorOn(void);
void HD44780_CursorOff(void);
void HD44780_ScrollLeft(void);
void HD44780_ScrollRight(void);
void HD44780_Scroll(uint8_t dir, uint8_t step, uint32_t delay);
void HD44780_CreateChar(uint8_t location ,const uint8_t *data);
void HD44780_PutCustom(uint8_t x, uint8_t y, uint8_t location);
void HD44780_CursorSet(uint8_t col, uint8_t row);
#ifdef HD44780_RW_Pin
uint8_t HD44780_Read4bit(void);
uint8_t HD44780_ReadData(void);
uint8_t HD44780_CheckBusy(void);
#endif

#endif

