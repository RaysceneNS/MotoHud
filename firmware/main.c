#include "main.h"

/*
Initialize the chip registers
*/
void init(void)
{
	// set port direction registers for writing to LCD high is OUT
	DDRC = 0b00110111;
	
	uart_init();
	LcdInitialise();
	
	//turn off unused parts to conserve power
	power_usi_disable();
	power_timer1_disable();
	power_twi_disable();
}

/*
function to initialize UART to communicate with the cell phone over a bluetooth serial connection

Serial connection is 9600 baud non-parity 8-bit.
*/
void uart_init (void)
{
	/*
	This is a 12-bit register which contains the USART baud rate. UBRRnH contains the four most significant bits, and
	UBRRnL contains the eight least significant bits of the USART baud rate.
	*/
	UBRR0H = (unsigned char)(BAUDRATE>>8);                      
	UBRR0L = (unsigned char)BAUDRATE;                           
	// 8bit data format
	UCSR0C |= (1<<UCSZ00) | (1<<UCSZ01);   	
	/*
	RXCIE0 - A USART Receive Complete interrupt will be generated only if the RXCIEn bit, the Global Interrupt Flag, and the RXCn bits are set.
	RXEN0 - Writing this bit to one enables the USART Receiver. When enabled, the receiver will override normal port operation for the RxDn pin.
	*/
	UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);                // enable receiver and transmitter, enable ISR for receive
}

/*
 Parses the instruction in the input data buffer
 each data buffer starts with a byte that denotes the instruction type
*/
void parse_data(uint8_t data_length, char message[16])
{
	unsigned char t = message[0];
	switch(t)
	{
		case 0x01: //turn indicator
		{
			directionType = message[1];
			directionAvailableAngles = message[2];
			directionOutAngle = message[3];
		}
		break;
		case 0x02: //lanes
		{
			lanesOutline = message[1];
			lanesArrow = message[2];
		}
		break;
		case 0x03: //distance
		{
			distanceThousands = message[1];
			distanceHundreds = message[2];
			distanceTens = message[3];
			distanceDecimal = message[4];
			distanceOnes = message[5];
			distanceUnit = message[6];
		}
		break;
		case 0x04: //camera
		{
//			speedCamera = message[1];
		}
		break;
		case 0x05: //time
		{
//			traffic = message[1];
			hourTens = message[2];
			hourOnes = message[3];
			colon = message[4];
			minuteTens = message[5];
			minuteOnes = message[6];
			flag = message[7];
		}
		break;
		case 0x06: //speed warning
		{
			speedHundreds = message[1];
			speedTens = message[2];
			speedOnes = message[3];
			slash = message[4];
			limitHundreds = message[5];
			limitTens = message[6];
			limitOnes = message[7];
			//isSpeeding = message[8];
			//isIcon = message[9];
		}
		break;
	}
}


/* Interrupt Service Routine for Receive Complete
NOTE: vector name changes with different AVRs see AVRStudio -
Help - AVR-Libc reference - Library Reference - <avr/interrupt.h>: Interrupts
for vector names other than USART_RXC_vect for ATmega32 

Thanks to Gabonator for his work on decoding the Garmin HUD protocol
https://github.com/gabonator/Work-in-progress/tree/master/GarminHud

The incoming packet format is

 ____________________________________________________________________________________________________
| 0x10 | 0x7b | pack len | data len | 0x00 | 0x00 | 0x00 | 0x55 | 0x15 | data... | crc | 0x10 | 0x03 |
 ----------------------------------------------------------------------------------------------------
| sync bytes  | len+6    | len      | data preamble                    | data [] | crc | tail bytes  |
 ----------------------------------------------------------------------------------------------------
*/

volatile uint8_t command_mode = FALSE; //command mode is set by issuing a 0x10;
volatile char state = STATE_WAIT;
volatile uint8_t rxindex = 0;
volatile char rx_buffer[32];

ISR(USART0_RX_vect)
{
	char value = UDR0;             //read UART register into value
	
	// detect the rising edge of a command mode state switch
	// commands are escaped with a 0x10 character, the next character that appears after the command mode is set
	// will determine the command to perform
	if(value == 0x10 && command_mode == FALSE)
	{
		command_mode = TRUE;
		return;
	}
	
	if(command_mode == TRUE)
	{
		//command mode only lives for a single byte
		command_mode = FALSE;
		
		//read the command byte
		switch(value)
		{
			case 0x10:	//to send 0x10 as data, it needs to be sent twice, once as a command mode switch and then as a data value
			break;

			case 0x7B:
			{
				//command is to start the packet
				rxindex = 0;
				for (int i = 0; i < 32;i++)
				{
					//We set the buffer to NULL, not 0.
					rx_buffer[i] = 0x00;
				}
				state = STATE_COUNT;
				return;
			}
			break;
			
			case 0x03:
			{
				//command is to end the packet, test to see if the buffer contains enough characters at this point
				if(rxindex >= 10 && STATE_COUNT == state)
				{
					// rx_buffer contains data from pack length to end bytes, we need to verify that the message is complete
					//determine the crc for this packet
					uint8_t crc = 0x7B;
					for(int jj = 0; jj < rxindex-1; jj++)
					{
						crc += rx_buffer[jj];
					}
					crc = (-(int)crc) & 0xff;
					
					//test the packet
					if(	rx_buffer[0]==(rx_buffer[1]+6) && //data length and packet length agree
						rx_buffer[rxindex-1] == crc && //crc agrees with data
						rx_buffer[2]==0x00 &&
						rx_buffer[3]==0x00 &&
						rx_buffer[4]==0x00 &&
						rx_buffer[5]==0x55 &&
						rx_buffer[6]==0x15)
					{
						//copy the verified message from the receive buffer to a new message buffer
						char message[16];
						for(int ii = 7; ii < (rxindex-1); ii++)
						{
							message[ii-7]=rx_buffer[ii];
						}
						
						// a good packet is ready, process it now
						parse_data(rxindex-8, message);
					}
				}
				state = STATE_WAIT;
				return;
			}
			break;
		}
	}
	
	if(STATE_COUNT == state)
	{
		rx_buffer[rxindex] = value;
		rxindex++;
	}
}

void LcdDrawLaneTurn(void)
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

void LcdDrawAvailableRoutes(void)
{
	//directionAvailableAngles contains the lanes available
	if((directionAvailableAngles & SharpRight)){
		LcdBitmap(ORDINAL_135);
	}
	if((directionAvailableAngles & Right)){
		LcdBitmap(ORDINAL_90);
	}
	if((directionAvailableAngles & EasyRight)){
		LcdBitmap(ORDINAL_45);
	}
	if((directionAvailableAngles & Straight)){
		LcdBitmap(ORDINAL_0);
	}
	if((directionAvailableAngles & EasyLeft)){
		LcdBitmap(ORDINAL_315);
	}
	if((directionAvailableAngles & Left)){
		LcdBitmap(ORDINAL_270);
	}
	if((directionAvailableAngles & SharpLeft)){
		LcdBitmap(ORDINAL_225);
	}
}

void LcdDrawRoundabout(void)
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
void LcdDrawTurnIndicators(void)
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
		LcdDrawLaneTurn();
		LcdDrawAvailableRoutes();
		break;
		
		case 0x04: // only use a single drawing type counter clock-wise (right hand drive) for the roundabout.
		case 0x08:
		LcdDrawRoundabout();
		LcdDrawAvailableRoutes();
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
void LcdDrawTime(void)
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
void LcdDrawSpeedWarning(void)
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
void LcdDrawDistance(void)
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
void LcdDrawLaneAssistArrows(void)
{
	LcdClearBlock(0,5,84,1);
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

void LcdBootAnimation()
{
	// animate a startup sequence, filling the display from the bottom to the top of the display with horizontal bars
	// The reason is to allow the ADC to stabilize the internal values after the boot
	for(int8_t y = 5; y >= 0; y--)
	{
		LcdGotoXY(21, y);
		for(uint8_t x=0; x < 42; x++) // fill half block 
		{
			LcdWrite(LCD_DATA, 0xf0);
		}
		_delay_ms(100);
			
		LcdGotoXY(21, y);
		for(uint8_t x=0; x < 42; x++) // fill full block
		{
			LcdWrite(LCD_DATA, 0xff);
		}
		_delay_ms(100);
	}
}

/*
Main entry point 
*/
int main(void)
{
	//initialize the chip registers
	init();
	// enable interrupt service routines, we need these for the ADC
	sei();

	LcdClear();
	LcdBootAnimation();
	
	// enable watch dog, careful that timeout is longer than loop sleep time
	wdt_enable(WDTO_250MS);

    // use the main loop to regularly update the display
	uint8_t isConnected = 0;
    while (1) 
    {
		//check the blue tooth state pin to determine if we are connected, clear the screen on state change
		if(PINA & (1<<PINA0))
		{
			if(!isConnected)
			{
				isConnected = 0x01;
				LcdClear();
			}
		}
		else
		{
			if(isConnected)
			{
				isConnected = 0x00;
				LcdClear();
			}	
		}
		
		if (isConnected)
		{
			//draw the display now to represent the state
			LcdDrawTurnIndicators();
			LcdDrawLaneAssistArrows();
			LcdDrawDistance();
			LcdDrawTime();
		}
		else
		{
			//draw the display now to represent the state
			LcdGotoXY(18,1);
			LcdString("Waiting\0");
			LcdGotoXY(32,2);
			LcdString("for\0");
			LcdGotoXY(10,3);
			LcdString("Bluetooth\0");
			LcdGotoXY(8,4);
			LcdString("Connection\0");
		}
		
	    _delay_ms(DELAY_MS);
	    wdt_reset();
    }
    return 0;
}

