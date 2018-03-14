#ifndef MAIN_H_
#define MAIN_H_

#define F_CPU 1000000UL         /* 8MHz crystal oscillator / div8 */
// define some macros
#define BAUD 9600                                   // define baud

#include <stdint.h>       // needed for uint8_t
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include "PCD8544.h"
#include "attiny1634_board.h"

#define DELAY_MS 25

#define BAUDRATE ((F_CPU)/(BAUD*16UL)-1)            // set baud rate value for UBRR

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

// state variables for the display
// direction
uint8_t directionType;
uint8_t directionRoundAboutOutAngle;
uint8_t directionOutAngle;
//lanes
uint8_t lanesArrow;
uint8_t lanesOutline;
//distance
uint8_t distanceThousands;
uint8_t distanceHundreds;
uint8_t distanceTens;
uint8_t distanceDecimal;
uint8_t distanceOnes;
uint8_t distanceUnit;
//camera
uint8_t speedCamera;
//time remain
uint8_t traffic;
uint8_t hourTens;
uint8_t hourOnes;
uint8_t colon;
uint8_t minuteTens;
uint8_t minuteOnes;
uint8_t flag;
//speed warning
uint8_t speedHundreds;
uint8_t speedTens;
uint8_t speedOnes;
uint8_t slash;
uint8_t limitHundreds;
uint8_t limitTens;
uint8_t limitOnes;
uint8_t isSpeeding;
uint8_t isIcon;
//gps label
uint8_t isGps;


// The inputted commands are never going to be
// more than 16 chars long. Volatile for the ISR.
volatile unsigned char rx_buffer[16];

void uart_init (void);
void init (void);

#endif /* MAIN_H_ */