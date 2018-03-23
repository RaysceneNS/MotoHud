#ifndef MAIN_H_
#define MAIN_H_

#define F_CPU 8000000UL         /* 8MHz crystal oscillator / div8 */
#define BAUD 38400              // define baud
#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1) // set baud rate value for UBRR

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
#include "drawing.h"

#define DELAY_MS 50 // the delay between screen updates 1000ms/50ms = 20fps

#define TRUE 1
#define FALSE 0

#define STATE_WAIT 0
#define STATE_BUFFER 1

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

uint8_t  screen_buff[200];

enum MessageType
{
	TurnIndicator = 0x01,
	Lanes = 0x02,
	Distance = 0x03,
	Camera = 0x04,
	Time = 0x05,
	SpeedWarn = 0x06,
	GpsLabel = 0x07
};

int main(void);
	
#endif /* MAIN_H_ */