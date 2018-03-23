#include "main.h"

/*
Initialize the chip registers
*/
void init(void)
{
	// set port direction registers for writing to LCD high is OUT
	DDRC = 0b00110111;
	
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
	
	//turn off unused parts to conserve power
	power_usi_disable();
	power_timer1_disable();
	power_twi_disable();
}

/*
 Parses the instruction in the input data buffer
 each data buffer starts with a byte that denotes the instruction type
*/
void parse_data(char message[16])
{
	unsigned char t = message[0];
	switch(t)
	{
		case TurnIndicator:
		{
			directionType = message[1];
			directionAvailableAngles = message[2];
			directionOutAngle = message[3];
		}
		break;
		case Lanes:
		{
			lanesOutline = message[1];
			lanesArrow = message[2];
		}
		break;
		case Distance:
		{
			distanceThousands = message[1];
			distanceHundreds = message[2];
			distanceTens = message[3];
			distanceDecimal = message[4];
			distanceOnes = message[5];
			distanceUnit = message[6];
		}
		break;
		case Time:
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
		case SpeedWarn: //speed warning
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
		//command mode only lives for a single cycle
		command_mode = FALSE;
		
		//read the command byte
		switch(value)
		{
			case 0x10:	//to send 0x10 as data, it needs to be sent twice, once as a command mode switch and then as a data value
			break;

			case 0x7B: //command is to start the packet
			{
				rxindex = 0;
				for (int i = 0; i < 32; i++)
				{
					//We set the buffer to NULL, not 0.
					rx_buffer[i] = 0x00;
				}
				state = STATE_BUFFER;
				return;
			}
			break;
			
			case 0x03: //command is to end the packet, test to see if the buffer contains enough characters at this point
			{
				if(rxindex >= 10 && STATE_BUFFER == state)
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
						rx_buffer[2]==0x00 &&
						rx_buffer[3]==0x00 &&
						rx_buffer[4]==0x00 &&
						rx_buffer[5]==0x55 &&
						rx_buffer[6]==0x15 &&
						rx_buffer[rxindex-1] == crc //crc agrees with data
						)
					{
						//copy the verified message from the receive buffer to a new message buffer
						char message[16];
						for(int ii = 7; ii < (rxindex-1); ii++)
						{
							message[ii-7]=rx_buffer[ii];
						}
						
						// a good packet is ready, process it now
						parse_data(message);
					}
				}
				state = STATE_WAIT;
				return;
			}
			break;
		
			default:
			{
				return;
			}
		}
	}
	
	if(STATE_BUFFER == state) //command byte is false and we're in the buffering state
	{
		rx_buffer[rxindex] = value;
		rxindex++;
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

	LcdInitialise();
	LcdClear();

	// animate a startup sequence, filling the display from the bottom to the top of the display with horizontal bars
	// The reason is to allow the ADC to stabilize the internal values after the boot
	for(int8_t y = 5; y >= 0; y--)
	{
		LcdGotoXY(21, y);
		for(uint8_t x=0; x < 42; x++) // fill full block
		{
			LcdWrite(LCD_DATA, 0xff);
		}
		_delay_ms(100);
	}
	
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
			DrawTurnIndicators(directionType, directionAvailableAngles, directionOutAngle);
			DrawLaneAssistArrows(lanesArrow, lanesOutline);
			DrawDistance(distanceThousands, distanceHundreds, distanceTens, distanceDecimal, distanceOnes, distanceUnit);
			DrawTime(flag, hourTens, hourOnes, colon, minuteTens, minuteOnes);
		}
		else
		{
			//draw the display now to represent the state
			DrawConnectionWait();
		}
		
	    _delay_ms(DELAY_MS);
	    wdt_reset();
    }
    return 0;
}