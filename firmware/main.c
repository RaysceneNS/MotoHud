#include "main.h"



void init(void)
{
	uart_init();	
}

// function to initialize UART
void uart_init (void)
{
	UBRR0H = (BAUDRATE>>8);                      // shift the register right by 8 bits
	UBRR0L = BAUDRATE;                           // set baud rate
	
	UCSR0B|= (1<<TXEN0)|(1<<RXEN0)|(1<<RXCIE0);                // enable receiver and transmitter, enable ISR for recieve
	UCSR0C|= (1<<UCSZ00)|(1<<UCSZ01);   // 8bit data format
}


// function to send data
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
		case 0x01: //direction
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

void drawDisplay(void)
{
	
}


int main(void)
{
	//initialize the chip registers
	init();

	// enable interrupt service routines, we need these for the ADC
	sei();

    /* Replace with your application code */
    while (1) 
    {
		if (command_ready == TRUE) 
		{
			//copy_command();
			//process_command();

			command_ready = FALSE;
		}
		
		//draw the display now to represent the state
		drawDisplay();
		
	    //delay to slow looping
	    _delay_ms(DELAY_MS);
    }
    return 0;
}

