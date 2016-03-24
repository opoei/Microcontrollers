#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart_functions_m48.h"
#include "lm73_functions_skel.h"
#include "twi_master.h"

char temp_val[16];

extern uint8_t lm73_rd_buf[2];
extern uint8_t lm73_wr_buf[2];
uint8_t main()
{
	//initialization
	uint16_t lm73_temp;
	init_twi();
	uart_init();

	//set LM73 mode for reading temperature by loading pointer register
	//this is done outside of the normal interrupt mode of operation 
	lm73_wr_buf[0] = LM73_PTR_TEMP; //load lm73_wr_buf[0] 
					//with temperature pointer address
	twi_start_wr(LM73_WRITE, lm73_wr_buf, 2); //start the TWI write process
	sei();             //enable interrupts to allow start_wr to finish

	while(1)
	{
		//read temperature data from LM73  (2 Bytes)
		twi_start_rd(LM73_READ, lm73_rd_buf, 2);
		_delay_ms(2);    //wait for it to finish

		//now assemble the two bytes read back into one 16-bit value
		lm73_temp = lm73_rd_buf[0]<<8;  //save high temperature byte 
		lm73_temp |= lm73_rd_buf[1];//"OR" in the low temp byte to lm73_temp 
		lm73_temp = lm73_temp >>7; //parse out decimal place
		itoa(lm73_temp, temp_val, 10); //convert to string in array 
		uart_puts(temp_val);
	}
}
