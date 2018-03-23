#include "drawing.h"

void DrawConnectionWait()
{
	LcdGotoXY(18,1);
	LcdString("Waiting\0");
	LcdGotoXY(32,2);
	LcdString("for\0");
	LcdGotoXY(10,3);
	LcdString("Bluetooth\0");
	LcdGotoXY(8,4);
	LcdString("Connection\0");
}

void DrawLaneTurn(uint8_t directionOutAngle)
{
	switch(directionOutAngle)
	{
		case SharpRight:
		LcdBitmap(DIR_135);
		break;
		case Right:
		LcdBitmap(DIR_90);
		break;
		case EasyRight:
		LcdBitmap(DIR_45);
		break;
		case Straight:
		LcdBitmap(DIR_0);
		break;
		case EasyLeft:
		LcdBitmap(DIR_315);
		break;
		case Left:
		LcdBitmap(DIR_270);
		break;
		case SharpLeft:
		LcdBitmap(DIR_225);
		break;
	}
}

void DrawAvailableRoutes(uint8_t available)
{
	//available contains the lanes available
	if((available & SharpRight)){
		LcdBitmap(ORDINAL_135);
	}
	if((available & Right)){
		LcdBitmap(ORDINAL_90);
	}
	if((available & EasyRight)){
		LcdBitmap(ORDINAL_45);
	}
	if((available & Straight)){
		LcdBitmap(ORDINAL_0);
	}
	if((available & EasyLeft)){
		LcdBitmap(ORDINAL_315);
	}
	if((available & Left)){
		LcdBitmap(ORDINAL_270);
	}
	if((available & SharpLeft)){
		LcdBitmap(ORDINAL_225);
	}
}

void DrawRoundabout(uint8_t directionOutAngle)
{
	//direction out angle is the direction to travel
	switch(directionOutAngle)
	{
		case SharpRight:
		LcdBitmap(RND_135);
		break;
		case Right:
		LcdBitmap(RND_90);
		break;
		case EasyRight:
		LcdBitmap(RND_45);
		break;
		case Straight:
		LcdBitmap(RND_0);
		break;
		case EasyLeft:
		LcdBitmap(RND_315);
		break;
		case Left: //Left
		LcdBitmap(RND_270);
		break;
		case SharpLeft:
		LcdBitmap(RND_225);
		break;
	}
}

/*
Print the turn indicator in the space of 38 width x 40 high
*/
void DrawTurnIndicators(uint8_t directionType, uint8_t directionAvailableAngles, uint8_t directionOutAngle)
{
	// the turn indicator is a three byte message [directionType]:[directionAvailableAngles]:[directionOutAngle]
	//clear the on screen memory buffer
	for(uint8_t i = 0; i < 200; i++)
	{
		screen_buff[i] = 0;
	}
	
	switch(directionType)
	{
		case 0x00: // Off
		// direction indicator off, clear the turn area
		LcdClearBlock(0,0,40,5);
		break;
		
		case 0x01: //Lane
		case 0x02: //Longer Lane
		DrawLaneTurn(directionOutAngle);
		DrawAvailableRoutes(directionAvailableAngles);
		break;
		
		case 0x04: // only use a single drawing type counter clock-wise (right hand drive) for the roundabout.
		case 0x08:
		DrawRoundabout(directionOutAngle);
		DrawAvailableRoutes(directionAvailableAngles);
		break;
		
		case 0x10: // only use a single drawing type (right hand drive) for the uturn.
		case 0x20:
		LcdBitmap(DIR_UTURN);
		break;
		
		case 0x80: // Arrows only
		break;
	}
	//copy the memory buffer to the screen position
	LcdBitBlt();
}

/*
draw the arrival time hh:mm
*/
void DrawTime(uint8_t flag, uint8_t hourTens, uint8_t hourOnes, uint8_t colon, uint8_t minuteTens, uint8_t minuteOnes)
{
	LcdClearBlock(40,4,44,1);
	//arrival flag
	if(flag){
		LcdGliff(40, 4, 10, CHECK_FLAG);
	}
	if(hourTens){
		LcdCharacter(54, 4, hourTens == 0x0A ? '0' : hourTens+'0');
	}
	if(hourOnes){
		LcdCharacter(60, 4, hourOnes == 0x0A ? '0' : hourOnes+'0');
	}
	// squish this character
	if(colon){
		LcdCharacter(66, 4, ':');
	}
	if(minuteTens){
		LcdCharacter(72, 4, minuteTens == 0x0A ? '0' : minuteTens+'0');
	}
	if(minuteOnes){
		LcdCharacter(78, 4, minuteOnes == 0x0A ? '0' :  minuteOnes+'0');
	}
}

/*
Print the speed warning
*/
void DrawSpeedWarning(uint8_t speedHundreds, uint8_t speedTens, uint8_t speedOnes, uint8_t slash,
	uint8_t limitHundreds, uint8_t limitTens, uint8_t limitOnes)
{
	LcdClearBlock(40,3,44,1);
	
	if(speedHundreds){
		LcdCharacter(42, 3, speedHundreds == 0x0A ? '0' : speedHundreds+'0');
	}
	if(speedTens){
		LcdCharacter(48, 3, speedTens == 0x0A ? '0' : speedTens+'0');
	}
	if(speedOnes){
		LcdCharacter(54, 3, speedOnes == 0x0A ? '0' : speedOnes+'0');
	}
	
	// squish this char
	if(slash){
		LcdCharacter(60, 3, '\\');
	}
	if(limitHundreds){
		LcdCharacter(66, 3, limitHundreds == 0x0A ? '0' : limitHundreds+'0');
	}
	if(limitTens){
		LcdCharacter(72, 3, limitTens == 0x0A ? '0' : limitTens+'0');
	}
	if(limitOnes){
		LcdCharacter(78, 3, limitOnes == 0x0A ? '0' : limitOnes+'0');
	}
	
	//todo speeding indication?
	//todo icon
}

/*
Display the distance to the next maneuver
*/
void DrawDistance(uint8_t distanceThousands, uint8_t distanceHundreds, uint8_t distanceTens, uint8_t distanceDecimal, uint8_t distanceOnes, uint8_t distanceUnit)
{
	LcdClearBlock(40,0,44,3);
	
	// each medium number is 16x12
	if(distanceDecimal)
	{
		if(distanceHundreds){
			LcdMediumDigit(43, 0, distanceHundreds);
		}
		if(distanceTens){
			LcdMediumDigit(55, 0, distanceTens);
		}
		LcdGliff(67, 1, 5, decimal);
		if(distanceOnes){
			LcdMediumDigit(72, 0, distanceOnes);
		}
	}
	else
	{
		if(distanceThousands){
			LcdMediumDigit(40, 0, distanceThousands);
		}
		
		if(distanceHundreds){
			LcdMediumDigit(51, 0, distanceHundreds);
		}
		if(distanceTens){
			LcdMediumDigit(62, 0, distanceTens);
		}
		if(distanceOnes){
			LcdMediumDigit(73, 0, distanceOnes);
		}
	}
	
	switch(distanceUnit)
	{
		case 0x01:
		LcdCharacter(77, 2, 'm');
		break;
		case 0x03:
		LcdCharacter(70, 2, 'k');
		LcdCharacter(77, 2, 'm');
		break;
		case 0x05:
		LcdCharacter(70, 2, 'm');
		LcdCharacter(77, 2, 'i');
		break;
	}
}

/*
Display the lane assist arrows
*/
void DrawLaneAssistArrows(uint8_t lanesArrow, uint8_t lanesOutline)
{
	LcdClearBlock(0, 5, 84, 1);
	//lanesOutline dots /outline/outline/outline/outline/outline/outline/ dots"
	//lanesArrow   none / arrow / arrow / arrow / arrow / arrow / arrow / none"
	// 2 10 2 |  2 10 2 |  2 10 2 |  2 10 2 |  2 10 2 |  2 10 2
	// as each position can only contain one gliff there is a preference order DOTS > FILLED > OUTLINE
	if((lanesOutline & 0b01000000)){
		LcdGliff(2, 5, 10, LANE_ARROW_OUTLINE);
	}
	if((lanesArrow & 0b01000000)){
		LcdGliff(2, 5, 10, LANE_ARROW_FILLED);
	}
	if((lanesArrow & 0b10000000)){
		LcdGliff(2, 5, 10, LANE_ARROW_DOTS);
	}

	if((lanesOutline & 0b00100000)){
		LcdGliff(16, 5, 10, LANE_ARROW_OUTLINE);
	}
	if((lanesArrow & 0b00100000)){
		LcdGliff(16, 5, 10, LANE_ARROW_FILLED);
	}

	if((lanesOutline & 0b00010000)){
		LcdGliff(30, 5, 10, LANE_ARROW_OUTLINE);
	}
	if((lanesArrow & 0b00010000)){
		LcdGliff(30, 5, 10, LANE_ARROW_FILLED);
	}

	if((lanesOutline & 0b00001000)){
		LcdGliff(44, 5, 10, LANE_ARROW_OUTLINE);
	}
	if((lanesArrow & 0b00001000)){
		LcdGliff(44, 5, 10, LANE_ARROW_FILLED);
	}

	if((lanesOutline & 0b00000100)){
		LcdGliff(58, 5, 10, LANE_ARROW_OUTLINE);
	}
	if((lanesArrow & 0b00000100)){
		LcdGliff(58, 5, 10, LANE_ARROW_FILLED);
	}

	if((lanesOutline & 0b00000010)){
		LcdGliff(72, 5, 10, LANE_ARROW_OUTLINE);
	}
	if((lanesArrow & 0b00000010)){
		LcdGliff(72, 5, 10, LANE_ARROW_FILLED);
	}
	if((lanesArrow & 0b00000001)){
		LcdGliff(72, 5, 10, LANE_ARROW_DOTS);
	}
}
