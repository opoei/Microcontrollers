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

#define DIGIT_ONE  0x00
#define DIGIT_TWO  0x10
#define DIGIT_COLON 0x20
#define DIGIT_THREE  0x30
#define	DIGIT_FOUR 0x40

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
int8_t debounce_switches(uint16_t *state, uint8_t i) {
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
void chk_buttons(uint16_t *state, uint16_t *num)
{

	uint8_t itr;
	for( itr=0; itr<8; itr++)
	{
		if( debounce_switches(state, itr))
		{
			*num += 1 << itr; // Could also use |= as that is what we are doing.
		}
	}
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
		_delay_ms(2);
		PORTA = 0xFF;
		PORTB = DIGIT_TWO;
		PORTA = itoseven(tens);
	}
	else if(num > 99 && num < 1000)
	{
		PORTB = DIGIT_ONE;
		PORTA = itoseven(ones);
		_delay_ms(2);
		PORTA = 0xFF;
		PORTB = DIGIT_TWO;
		PORTA = itoseven(tens);
		_delay_ms(2);
		PORTA = 0xFF;
		PORTB = DIGIT_THREE;
		PORTA = itoseven(hundreds);
	}	
	else if (num >999)
	{
		PORTB = DIGIT_ONE;
		PORTA = itoseven(ones);
		_delay_ms(2);
		PORTA = 0xFF;
		PORTB = DIGIT_TWO;
		PORTA = itoseven(tens);
		_delay_ms(2);
		PORTA = 0xFF;
		PORTB = DIGIT_THREE;
		PORTA = itoseven(hundreds);
		_delay_ms(2);
		PORTA = 0xFF;
		PORTB = DIGIT_FOUR;
		PORTA = itoseven(thousands);
	}	
}

uint8_t main()
{
	DDRB = 0xF0; //set port bits 4-7 B as outputs
	uint16_t num = 0;
	uint16_t state[8];
	//initialize array values for debouncing
	for(int i=8; i>0; --i)
	{
		state[i]= 0;
	}
	while(1)
	{
		//make PORTA an input port with pullups 
		DDRA = 0x00;	
		PORTA = 0xFF;
		//enable tristate buffer for pushbutton switches
		PORTB = 0x70; 
		//check the buttons. Increment by appropriate value if switch is active.
		chk_buttons(state, &num); 
		//rollover at 1023
		if(num > 1023)	
			num = 1;
		//switch PORTA to output
		DDRA = 0xFF;  
		//Send num to display
		segsum(num); 
		_delay_ms(2);
	}
	return 0;
}
