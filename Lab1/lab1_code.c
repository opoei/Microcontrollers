// Oliver Poei

//This program increments a binary display of the number of button pushes on switch 
//S1 on the mega128 board.

#include <avr/io.h>
#include <util/delay.h>

//*******************************************************************************
//                            debounce_switch                                  
// Adapted from Ganssel's "Guide to Debouncing"            
// Checks the state of pushbutton S1 It shifts in ones till the button is pushed. 
// Function returns a 1 only once per debounced button push so a debounce and toggle 
// function can be implemented at the same time.  Expects active low pushbutton on 
// Port D bit zero.  Debounce time is determined by external loop delay times 12. 
//*******************************************************************************
int8_t debounce_switch() {
  static uint16_t state = 0; //holds present state
  state = (state << 1) | (! bit_is_clear(PIND, 0)) | 0xE000;
  if (state == 0xF000) return 1;
  return 0;
}


void led_split(int8_t num) {
    int8_t ones, tens;

    ones = num % 10;
    tens = (num / 10) % 10;
    
    /* Set the LEDs to show the BCD number */
    PORTB = ones | (tens << 4);
}
//*******************************************************************************
// Check switch S0.  When found low for 12 passes of "debounc_switch(), increment
// PORTB.  This will make an incrementing count on the port B LEDS. 
//*******************************************************************************
int main()
{
int i = 0;
DDRB = 0xFF;  //set port B to all outputs
while(1){     //do forever
 if(debounce_switch()) 
 	{
		if(i > 99) {i=0;}
		else{i++;}
		led_split(i);
	}  
  _delay_ms(2);                    //keep in loop to debounce 24ms
  } //while 
} 
