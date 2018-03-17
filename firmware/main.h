#ifndef MAIN_H_
#define MAIN_H_

#define F_CPU 8000000UL         /* 8MHz crystal oscillator / div8 */
#define BAUD 38400              // define baud

#include <stdint.h>       // needed for uint8_t
#include <stdlib.h>       // needed for itoa
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include "PCD8544.h"
#include "attiny1634_board.h"
#include "bitmap.h"

#define DELAY_MS 40 // the delay between screen updates 1000ms/40ms = 25fps

#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)            // set baud rate value for UBRR

// define some macros for bit manipulations
#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))

#define TRUE 1
#define FALSE 0

#define STATE_WAIT 0
#define STATE_COUNT 1
#define STATE_ACCUMULATE_DATA 2
#define STATE_CHECKSUM 3
#define STATE_CHECK_PREAMBLE 4
#define STATE_MESSAGE_READY 5

// Next turn indicators
volatile uint8_t directionType;
volatile uint8_t directionAvailableAngles;
volatile uint8_t directionOutAngle;
//lanes
volatile uint8_t lanesArrow;
volatile uint8_t lanesOutline;
//distance
volatile uint8_t distanceThousands;
volatile uint8_t distanceHundreds;
volatile uint8_t distanceTens;
volatile uint8_t distanceDecimal;
volatile uint8_t distanceOnes;
volatile uint8_t distanceUnit;
//camera
//volatile uint8_t speedCamera;
//time remain
//volatile uint8_t traffic;
volatile uint8_t hourTens;
volatile uint8_t hourOnes;
volatile uint8_t colon;
volatile uint8_t minuteTens;
volatile uint8_t minuteOnes;
volatile uint8_t flag;
//speed warning
volatile uint8_t speedHundreds;
volatile uint8_t speedTens;
volatile uint8_t speedOnes;
volatile uint8_t slash;
volatile uint8_t limitHundreds;
volatile uint8_t limitTens;
volatile uint8_t limitOnes;
//volatile uint8_t isSpeeding;
//volatile uint8_t isIcon;

uint8_t  screen_buff[200];

void uart_init (void);
void init (void);
int main(void);

void LcdDrawLaneTurn(void);
void LcdDrawRoundabout(void);
void LcdDrawTurnIndicators(void);
void LcdDrawTime(void);
void LcdDrawSpeedWarning(void);
void LcdDrawCamera(void);
void LcdDrawDistance(void);
void LcdDrawLaneAssistArrows(void);
void LcdDrawGpsLabel(void);
void LcdDrawDisplay(void);

enum State 
{
	SharpRight = 0x02,
	Right = 0x04,
	EasyRight = 0x08,
	Straight = 0x10,
	EasyLeft = 0x20,
	Left = 0x40,
	SharpLeft = 0x80,
};
	
#endif /* MAIN_H_ */