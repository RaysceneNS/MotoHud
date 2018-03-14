#include "main.h"

/*
Initialize the chip registers
*/
void init(void)
{
	// TODO set port direction registers high is OUT
	
	uart_init();
	LcdInitialise();
	
	//turn off unused parts to conserve power
	//power_usi_disable();
	power_timer1_disable();
}

/*
function to initialize UART to communicate with the cell phone over a bluetooth serial connection

Serial connection is 9600 baud non-parity 8-bit.
*/
void uart_init (void)
{
	UBRR0H = (BAUDRATE>>8);                      // shift the register right by 8 bits
	UBRR0L = BAUDRATE;                           // set baud rate
	
	UCSR0B|= (1<<TXEN0)|(1<<RXEN0)|(1<<RXCIE0);                // enable receiver and transmitter, enable ISR for receive
	UCSR0C|= (1<<UCSZ00)|(1<<UCSZ01);   // 8bit data format
}

/*
function to send data over the UART port
*/
void uart_transmit (unsigned char data)
{
	while (!( UCSR0A & (1<<UDRE0)));                // wait while register is free
	UDR0 = data;                                   // load data in the register
}

/*
Data payload has the following format
*/
void parse_data(uint8_t data_length, unsigned char rx_buffer[16])
{
	unsigned char t = rx_buffer[0];
	switch(t)
	{
		case 0x01: //turn indicator
		{
			directionType = rx_buffer[1];
			directionRoundAboutOutAngle = rx_buffer[2];
			directionOutAngle = rx_buffer[3];
		}
		break;
		case 0x02: //lanes
		{
			lanesArrow = rx_buffer[1];
			lanesOutline = rx_buffer[2];			
		}
		break;
		case 0x03: //distance
		{
			distanceThousands = rx_buffer[1];
			distanceHundreds = rx_buffer[2];
			distanceTens = rx_buffer[3];
			distanceDecimal = rx_buffer[4];
			distanceOnes = rx_buffer[5];
			distanceUnit = rx_buffer[6];			
		}
		break;
		case 0x04: //camera
		{
			speedCamera = rx_buffer[1];
		}
		break;
		case 0x05: //time
		{
			traffic = rx_buffer[1];
			hourTens = rx_buffer[2];
			hourOnes = rx_buffer[3];
			colon = rx_buffer[4];
			minuteTens = rx_buffer[5];
			minuteOnes = rx_buffer[6];
			flag = rx_buffer[7];
		}
		break;
		case 0x06: //speed warning
		{
			speedHundreds = rx_buffer[1];
			speedTens = rx_buffer[2];
			speedOnes = rx_buffer[3];
			slash = rx_buffer[4];
			limitHundreds = rx_buffer[5];
			limitTens = rx_buffer[6];
			limitOnes = rx_buffer[7];
			isSpeeding = rx_buffer[8];
			isIcon = rx_buffer[9];
		}
		break;
		case 0x07: //gps label
		{
			isGps = rx_buffer[1];
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
ISR(USART0_RX_vect)
{
	static char state = STATE_WAIT;
	static char byte_count;
	static char data_length;
	static char preamble_count;
	static char checksum;
	static unsigned char *buff;	
	static uint8_t command_mode = FALSE; //command mode is set by issuing a 0x10;
	
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
				byte_count = 0;
				state = STATE_COUNT;
				return;
			}
			break;
			
			case 0x03:
			{
				//command is to end the packet
				if(state == STATE_MESSAGE_READY)
				{
					// a good packet is ready, send it now
					parse_data(data_length, buff);
				}
				state = STATE_WAIT;
				return;
			}
			break;
		}	
	}
	
	
	// use a state machine to parse the incoming packet
	switch(state)
	{
		case STATE_COUNT: //got head sync, get byte count
		{
			if(byte_count == 0)
			{
				byte_count = value;
			}
			else
			{
				if(byte_count == (value+6))
				{
					data_length = value;
					preamble_count = 5;
					state = STATE_CHECK_PREAMBLE;
				}
				else
				{
					//reset back if lengths don't match
					state = STATE_WAIT;
				}
			}
		}
		break;
		
		case STATE_CHECK_PREAMBLE://got byte count, now check for preamble to match defined sequence
		{
			*buff = value;
			buff++;
			preamble_count--;
			
			if (preamble_count == 0)
			{
				if(buff[0] == 0x00 && 
				   buff[1] == 0x00 &&
				   buff[2] == 0x00 &&
				   buff[3] == 0x55 &&
				   buff[4] == 0x15)
				{
					 checksum = 0;
					 buff = rx_buffer;
					 state = STATE_ACCUMULATE_DATA;
				}
				else
				{
					//reset back if preamble incorrect
					state = STATE_WAIT;	
				}
			}
		}
		break;
		
		case STATE_ACCUMULATE_DATA: //pre amble bytes match, accumulate data into buffer
		{
			*buff = value;
			checksum += *buff;
			buff++;
			byte_count--;
			if (byte_count == 0)
			{
				state = STATE_CHECKSUM;
			}
		}
		break;

		case STATE_CHECKSUM: //got data bytes, test checksum
		{
			//CRC = (65535 - CRC) + 1;
			if (checksum == value)
			{
				//crc check is good, look for tail 
				state = STATE_MESSAGE_READY;
			}
			else
			{
				//message bad, reset
				state = STATE_WAIT;
			}
		}
		break;
	}
	
	//prev_value = value;
	command_mode = FALSE;
}

/*
Print the turn indicator in the space of 38 width x 40 high
*/
void LcdDrawTurnIndicators()
{
	//directionOutAngle
	//directionRoundAboutOutAngle
	//directionType
	
	switch(directionType)
	{
		case 0x02: //SharpRight
		LcdBitmap(0,0,38, 40, DIR_135);
		break;
		case 0x04: //Right
		LcdBitmap(0,0,38, 40, DIR_90);
		break;
		case 0x08: //EasyRight
		LcdBitmap(0,0,38, 40, DIR_45);
		break;
		case 0x10: //straight
		LcdBitmap(0,0,38, 40, DIR_0);
		break;
		case 0x20: //EasyLeft
		LcdBitmap(0,0,38, 40, DIR_315);
		break;
		case 0x40: //Left
		LcdBitmap(0,0,38, 40, DIR_270);
		break;
		case 0x80: //SharpLeft
		LcdBitmap(0,0,38, 40, DIR_225);
		break;
	}
}

/*
draw the arrival time hh:mm 
*/
void LcdDrawTime()
{
	////traffic flag
	//if(traffic == 0x01){
		//LcdGliff(40, 4, 9, TRAFFIC);
	//}
	//arrival flag
	if(flag == 0x01){
		LcdGliff(40, 4, 9, CHECK_FLAG);
	}
	
	LcdCharacter(84-28, 4, hourTens);
	LcdCharacter(84-22, 4, hourOnes);
	
	// squish this char 
	if(colon == 0x01)
	{
		LcdCharacter(84-16, 4, ':');
	}
	
	LcdCharacter(84-12, 4, minuteTens);
	LcdCharacter(84-6, 4, minuteOnes);
}

/*
Print the speed warning 
*/
void LcdDrawSpeedWarning()
{
	LcdCharacter(84-40, 3, speedHundreds);
	LcdCharacter(84-34, 3, speedTens);
	LcdCharacter(84-28, 3, speedOnes);
	
	// squish this char
	if(slash == 0xff)
	{
		LcdCharacter(84-22, 3, '\\');
	}
	
	LcdCharacter(84-18, 3, limitHundreds);
	LcdCharacter(84-12, 3, limitTens);
	LcdCharacter(84-6, 3, limitOnes);
	
	//todo speeding indication?
	//todo icon
}

/*
Draw the speed camera
*/
void LcdDrawCamera()
{
	if(speedCamera == 0x01)
	{
		
	}
}

/*
Display the distance to the next maneuver
*/
void LcdDrawDistance()
{
	//distanceThousands
	//distanceHundreds
	//distanceTens
	//distanceDecimal
	//distanceOnes
	//distanceUnit
	
	// each medium number is 16x12	
	if(distanceDecimal == 0x01)
	{
		LcdMediumDigit(43, 0, distanceHundreds);
		LcdMediumDigit(55, 0, distanceTens);
		LcdGliff(67, 0, 5, decimal);
		LcdMediumDigit(72, 0, distanceOnes);
	}
	else
	{
		LcdMediumDigit(36, 0, distanceThousands);
		LcdMediumDigit(48, 0, distanceHundreds);
		LcdMediumDigit(60, 0, distanceTens);
		LcdMediumDigit(72, 0, distanceOnes);
	}
	
	LcdGotoXY(62, 2);
	switch(distanceUnit)
	{
		case 0x01:
		LcdString("m");
		break;
		case 0x03:
		LcdString("km");
		break;
		case 0x05:
		LcdString("m1");
		break;
	}
}

/*
Display the lane assist arrows

*/
void LcdDrawLaneAssistArrows()
{
	//lanesOutline dots /outline/outline/outline/outline/outline/outline/ dots"
	//lanesArrow   none / arrow / arrow / arrow / arrow / arrow / arrow / none"
	
	// 2 10 2 |  2 10 2 |  2 10 2 |  2 10 2 |  2 10 2 |  2 10 2
	if(lanesArrow && 0b10000000 == 0b10000000){
		LcdGliff(2, 5, 10, LANE_ARROW_DOTS);
	}

	if(lanesArrow && 0b01000000 == 0b01000000){
		LcdGliff(2, 5, 10, LANE_ARROW_FILLED);
	}
	if(lanesOutline && 0b01000000 == 0b01000000){
		LcdGliff(2, 5, 10, LANE_ARROW_OUTLINE);
	}

	if(lanesArrow && 0b00100000 == 0b00100000){
		LcdGliff(16, 5, 10, LANE_ARROW_FILLED);
	}
	if(lanesOutline && 0b00100000 == 0b00100000){
		LcdGliff(16, 5, 10, LANE_ARROW_OUTLINE);
	}

	if(lanesArrow && 0b00010000 == 0b00010000){
		LcdGliff(30, 5, 10, LANE_ARROW_FILLED);
	}
	if(lanesOutline && 0b00010000 == 0b00010000){
		LcdGliff(30, 5, 10, LANE_ARROW_OUTLINE);
	}

	if(lanesArrow && 0b00001000 == 0b00001000){
		LcdGliff(44, 5, 10, LANE_ARROW_FILLED);
	}
	if(lanesOutline && 0b00001000 == 0b00001000){
		LcdGliff(44, 5, 10, LANE_ARROW_OUTLINE);
	}

	if(lanesArrow && 0b00000100 == 0b00000100){
		LcdGliff(58, 5, 10, LANE_ARROW_FILLED);
	}
	if(lanesOutline && 0b00000100 == 0b00000100){
		LcdGliff(58, 5, 10, LANE_ARROW_OUTLINE);
	}

	if(lanesArrow && 0b00000010 == 0b00000010){
		LcdGliff(72, 5, 10, LANE_ARROW_FILLED);
	}
	if(lanesOutline && 0b00000010 == 0b00000010){
		LcdGliff(72, 5, 10, LANE_ARROW_OUTLINE);
	}
		
	if(lanesArrow && 0b00000001 == 0b00000001){
		LcdGliff(72, 5, 10, LANE_ARROW_DOTS);
	}
}

/*
Display the GPS label
*/
void LcdDrawGpsLabel()
{
	if(isGps==0x01)
	{
		//display the gps label?
	}
}

/*
Display the current machine state on the LCD display
*/
void LcdDrawDisplay(void)
{
	LcdDrawTurnIndicators();
	LcdDrawLaneAssistArrows();
	LcdDrawDistance();
	LcdDrawCamera();
	LcdDrawTime();
	LcdDrawSpeedWarning();
	LcdDrawGpsLabel();
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

	// animate a startup sequence, filling the display from the bottom to the top of the display with horizontal bars
	// The reason is to allow the ADC to stabilize the internal values after the boot
	for(uint8_t y = 5; y >= 0; y--)
	{
		for(uint8_t x=21; x < 63; x++) // top
		{
			LcdGotoXY(x, y);
			LcdWrite(LCD_DATA, 0xff);
		}
		_delay_ms(250);
	}
	
	// enable watch dog, careful that timeout is longer than loop sleep time
	wdt_enable(WDTO_250MS);

    // use the main loop to regularly update the display
    uint8_t i = 0xff;
    while (1) 
    {
		if(i++ == 0)
		{
			// every 255 loops interject a full screen clear approx every 6.5 secs
			LcdClear();
		}
		
		//check the blue tooth state pin to determine if we are connected
		if (PINA & (1<<PINA0))
		{
			//draw the display now to represent the state
			LcdDrawDisplay();				
		}
		else
		{
			//draw the display now to represent the state
			LcdGotoXY(0,0);
			LcdString("Waiting");
			LcdGotoXY(2,24);
			LcdString("for");
			LcdGotoXY(4,0);
			LcdString("Connection");
		}
		
	    //delay to slow looping
	    _delay_ms(DELAY_MS);
	    wdt_reset();
    }
    return 0;
}

