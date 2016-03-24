// lab2.c 
//Oliver Poei
//
//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define DIGIT_ONE  0x00
#define DIGIT_TWO  0x10
#define DIGIT_COLON 0x20
#define DIGIT_THREE  0x30
#define	DIGIT_FOUR 0x40

volatile uint8_t flag = 0;

/***********************************************************************/
//                            spi_init                               
//Initalizes the SPI port on the mega128. Does not do any further   
//external device specific initalizations.  Sets up SPI to be:                        
// clock=clk/2, cycle half phase, low polarity, MSB first
//interrupts disabled, poll SPIF bit in SPSR to check xmit completion
/***********************************************************************/
void spi_init(void){
  SPCR  |=   (1<<SPE | 1<<MSTR);           //set up SPI mode
  SPSR  |=   (1<<SPI2X); 		// double speed operation
 }

/***********************************************************************/
//                              tcnt0_init                             
//Initalizes timer/counter0 (TCNT0). TCNT0 is running in async mode
//with external 32khz crystal.  Runs in normal mode with no prescaling.
//Interrupt occurs at overflow 0xFF.
//
void tcnt0_init(void){
  TIMSK  |=  (1<<TOIE0);  //enable timer/counter0 overflow interrupt
  TCCR0  |=  ((1<<CS02) |(1<<CS00) );  
}
//******************************************************************************
//                           debounce_switches 
//Checks the state of the button number passed to it. It shifts in ones till   
//the button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//external loop delay times 12. 
//Saves status of button (i) into a state array.
//******************************************************************************
int8_t debounce_switches( uint16_t *state, uint8_t i) {
  state[i] = (state[i] << 1) | (! bit_is_clear(PINA, i)) | 0xE000;
  if (state[i] == 0xF000) return 1;
  return 0;
}

//******************************************************************************
//                          chk_buttons 
//Checks the buttons. Calls debounce_switches in a loop passing both the state array
//and the current switch being checked. 
//If debounce_switches returns a 1 for a switch, case statements determine which switch
//was activated, and increments count by the appropriate value.
//******************************************************************************

void chk_buttons(uint16_t *state, uint16_t *num, uint8_t *multiplier)
{
	uint8_t itr;
	for( itr=0; itr<8; itr++)
	{
		if( debounce_switches(state, itr))
		{
			*multiplier &= 0x0FE;
			switch(itr)
			{
				case 0:
					*multiplier ^= 0x02;
					if(*multiplier == 0x00) *multiplier = 0x01;
					break;
				case 1:
					*multiplier ^= 0x04;
					if(*multiplier == 0x00) *multiplier = 0x01;
					break;
			}
						
		}
	}
}


//******************************************************************************
//                         spi_read 
//reads spi data. Set PL to active low, SPCR to slave. Write dummy data to 
//initialize a clock. 
//
//	 	 	 	 	 	 
//******************************************************************************

uint8_t spi_read()
{
	PORTB |= 0x01; //PL is active low, setting it high disables parelel input.
	PORTE = 0x00; //clock enable
	SPDR = 0x00; //dummy write
	while(bit_is_clear(SPSR, SPIF)){};
	PORTE = 0x40; // reset clock enable to high 
	return (~SPDR);


}

//******************************************************************************
//                        encoder_direction 
// Determines the encoder directions.
// States:
//	 * 00  |    CCW
//	 * 01  |     ^
//	 * 11  |     |
//	 * 10  V     |
//	 * 00 CW     |
//
//******************************************************************************
int8_t encoder_direction(uint16_t *num, uint8_t *multiplier)
{
 	static uint8_t encoder = 0;
	uint8_t current = 0;
	uint8_t previous = 0;
	encoder = (encoder<<4);
	encoder |= spi_read();
	//left encoder
	current  = ((encoder &  0b00000011));
	previous = ((encoder & 0b00110000) >> 4);
	if((previous == 0b10) && (current == 0b00))
		*num += *multiplier ;	
	if((previous == 0b01) && (current == 0b00))
		*num -= *multiplier;
	//right encoder	
	current  = ((encoder &  0b00001100) >>2);
	previous = ((encoder & 0b11000000) >> 6);
	if((previous == 0b10) && (current == 0b00))
	 	*num += *multiplier;	
	if((previous == 0b01) && (current == 0b00))
		*num -= *multiplier;
	return 0;
}

void bar_graph(uint8_t display)
{
	SPDR = display;
       	while(bit_is_clear(SPSR, SPIF));
	PORTD = 0x04; //strobe output data reg (regclk)
	PORTD = 0x00;

}

//******************************************************************************
//                         itoseven 
// Accepts a number from 0 -9 Returns a hex value to display on the seven segment..
//******************************************************************************

uint8_t itoseven(uint8_t num)
{
    const uint8_t kLookup[10] = 
             {~0x3F, ~0x06, ~0x5B, ~0x4F, ~0x66, 
              ~0x6D, ~0x7D, ~0x07, ~0x7F, ~0x6F };
    return kLookup[num];
                      
}
 
//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and displays the result to the LED board.
//***********************************************************************************
void segsum(uint16_t num) {
	//could probably reduce memory usage by making this an array.
	uint8_t ones;
	uint8_t tens;
	uint8_t hundreds;
	uint8_t thousands;
  //break up decimal sum into 4 digit-segments
	ones = num % 10;
	tens = (num/10)%10;
	hundreds = (num/100)%10;
	thousands = (num/1000)%10;
  //determine how many digits there are
 	if(num < 10) 
	{
		PORTB = DIGIT_ONE;
		PORTA = itoseven(num);
	}
	else if(num > 9 && num <100)	
	{
		PORTB = DIGIT_ONE;
		PORTA = itoseven(ones);
		_delay_ms(1);
		PORTA = 0xFF;
		PORTB = DIGIT_TWO;
		PORTA = itoseven(tens);
	}
	else if(num > 99 && num < 1000)
	{
		PORTB = DIGIT_ONE;
		PORTA = itoseven(ones);
		_delay_ms(1);
		PORTA = 0xFF;
		PORTB = DIGIT_TWO;
		PORTA = itoseven(tens);
		_delay_ms(1);
		PORTA = 0xFF;
		PORTB = DIGIT_THREE;
		PORTA = itoseven(hundreds);
	}	
	else if (num >999)
	{
		PORTB = DIGIT_ONE;
		PORTA = itoseven(ones);
		_delay_ms(1);
		PORTA = 0xFF;
		PORTB = DIGIT_TWO;
		PORTA = itoseven(tens);
		_delay_ms(1);
		PORTA = 0xFF;
		PORTB = DIGIT_THREE;
		PORTA = itoseven(hundreds);
		_delay_ms(1);
		PORTA = 0xFF;
		PORTB = DIGIT_FOUR;
		PORTA = itoseven(thousands);
	}	
}

ISR(TIMER0_OVF_vect)
{
	flag = 1;
}

uint8_t main()
{
	int16_t num = 0;
	uint8_t  multiplier = 1;
	uint16_t state[8];
	tcnt0_init();
	DDRB = 0xF7; //enable output 4-7
	DDRE = 0x40; //enable portE output pin 6. (Enable for encoder shift reg)
	DDRD = 0x04;
	PORTE = 0x40;
	spi_init();
	sei();	
	while(1)
	{
		if(flag)
		{
			//make PORTA an input port with pullups 
			DDRA = 0x00;	
			PORTA = 0xFF;
			//enable tristate buffer for pushbutton switches
			PORTB = 0x70; 
			//check the buttons. Increment by appropriate value if switch is active.
			chk_buttons(state, &num, &multiplier); 
			DDRA = 0xFF;
			flag = 0;
		}
		if(multiplier == 0x06) multiplier = 0; //if both buttons are toggled, do not increment
		bar_graph(multiplier);
		encoder_direction(&num, &multiplier);
		//Send num to display
		if(num > 1023) num = 0;
		if(num < 0) num = 1023;
		segsum(num); 
		_delay_ms(1);
	}
	return 0;
}
