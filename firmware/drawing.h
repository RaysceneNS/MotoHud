/*
 * drawing.h
 *
 * Created: 2018-03-23 8:57:39 AM
 *  Author: Racine
 */ 
#ifndef DRAWING_H_
#define DRAWING_H_

#include <stdint.h>       // needed for uint8_t
#include "PCD8544.h"

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

void DrawConnectionWait();
void DrawTurnIndicators(uint8_t directionType, uint8_t directionAvailableAngles, uint8_t directionOutAngle);
void DrawTime(uint8_t flag, uint8_t hourTens, uint8_t hourOnes, uint8_t colon, uint8_t minuteTens, uint8_t minuteOnes);
void DrawDistance(uint8_t distanceThousands, uint8_t distanceHundreds, uint8_t distanceTens, uint8_t distanceDecimal, uint8_t distanceOnes, uint8_t distanceUnit);
void DrawLaneAssistArrows(uint8_t lanesArrow, uint8_t lanesOutline);

#endif /* DRAWING_H_ */