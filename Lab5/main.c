//Lab5
//Main File for the Mega128
                            
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include "lcd_functions.h"
#include "uart_functions.h"
#include "lm73_functions_skel.h"
#include "twi_master.h"

#define DIGIT_ONE  0x00
#define DIGIT_TWO  0x10
#define DIGIT_COLON 0x20
#define DIGIT_THREE  0x30
#define	DIGIT_FOUR 0x40

//define modes of operaton
#define DEFAULT 0x00
#define CLOCK_SET 0x01
#define ALARM_SET 0x02
#define ALARM_ON 0x04
#define SNOOZE_ON 1
char lcd_string_array[5];

//delclare the 2 byte TWI read and write buffers (lm73_functions_skel.c)
extern uint8_t lm73_rd_buf[2];
extern uint8_t lm73_wr_buf[2];

//initialize global variables
volatile uint8_t alarm_set = 0;
volatile uint8_t colon = 0;
volatile uint8_t  mode = 0x00;
volatile uint16_t state[8];
volatile uint8_t adc_isr_count =0 ;
volatile uint8_t snooze =0;
volatile uint8_t snooze_timer = 0;
volatile uint8_t isr_timer2_count  = 0;
volatile uint8_t lcd_lock = 0;

//define time struct for alarm and clock
typedef struct 
{
	int8_t hr;
	int8_t min;
	int8_t sec;
} time;

time clock = {12,30,55};
time *clock_ptr = &clock;
time alarm = {12,31,0};
time *alarm_ptr = &alarm;

//prototypes
void rollover();
void silence_alarm();

//initialization functions
void spi_init(void){
    DDRB |=  0x07;  //Turn on SS, MOSI, SCLK
    //mstr mode, sck=clk/2, cycle 1/2 phase, low polarity, MSB 1st, no interrupts 
    SPCR=(1<<SPE) | (1<<MSTR); //enable SPI, clk low initially, rising edge sample
    SPSR=(1<<SPI2X); //SPI at 2x speed (8 MHz)  
}

void tcnt0_init(void)
//Counter for clock, int occurs every sec
{
	ASSR  |= (1<<AS0);
	TIMSK |= (1<<OCIE0);  //enable timer/counter0 compare match interrupt
	TCCR0 |= ((1<<WGM01) | (1<<CS02) | (1<<CS01) | (1<<CS00 ));  
	OCR0  |= 0x1F;
}

void tcnt1_init()
//Generates (audio) signal for Alarm
{
	TIMSK |= (1<<OCIE1A); //set tcnt1 compare match
	TCCR1A = 0; //normal mode
	TCCR1B |= (1<<WGM12); //CTC at TOP
	OCR1A = 0x00FF; //TOP value, adjusts frequency
	DDRD |= (1<<PD7);  //set PD7 as output
}

void tcnt2_init(void){
	TIMSK |= (1<<TOIE2); //enable counter 2 compare match interrupt
	TCCR2 |= ((1<<WGM21) | (1<<WGM20) //enable Fast PWM 
		 |(1<<COM21) | (1<<COM20) //some shit that makes this work
		 |(1<<CS22) ); //prescale by 256
	OCR2 = 255;
	
}

void tcnt3_init()
{
	//set fast pwm
	TCCR3A |= (1<<WGM30); 
	TCCR3A |= (1<<COM3A1); //set output compare low
        TCCR3B |= (1<<WGM32);
	TCCR3B  |= (1<<CS30); //no prescale
	DDRE |= (1<<PE3);

	OCR3A = 0; //default 0%
}

void adc_init()
{
	DDRF = ~(1<<PF7); //set f7 to input
	PORTF = ~(1<<PF7); //no pull up

	ADCSRA |= (1<<ADEN); //enable ADC
	ADMUX  |= (1<<REFS0); //set vref to vcc
	ADMUX  |= ( (1<<MUX2) | (1<<MUX1) | (1<<MUX0) // single ended input to F7	
		   | (1<<ADLAR)); //Left justify ADC data
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) //prescale adc to 125kHz
		   | (1<< ADIE) |(1<<ADFR) //enable interrupt, set ADC to free run.
		   | (1<<ADSC); //start conversion
}

ISR(TIMER0_COMP_vect)
//Handles time counting, blinks colon every sec, checks snooze?
{
	colon ^= 0x01; //toggle colon
	clock_ptr->sec +=1;
	rollover();
		
	if(snooze == SNOOZE_ON)
	{
	OCR3A = 0;
		snooze_timer++;
		if(snooze_timer == 10)	
		{
			OCR3A = 120;
			snooze_timer = 0;
			snooze = 0;
		}
	}
}
ISR(TIMER1_COMPA_vect)
{
	PORTD ^=1<<PD7;
}
ISR(TIMER2_OVF_vect)
{
	if((isr_timer2_count %100 == 0)&& (lcd_lock == 0))
	{
		//make PORTA an input port with pullups 
		DDRA = 0x00;	
		PORTA = 0xFF;
		//enable tristate buffer for pushbutton switches
		PORTB = 0x70; 
		//check the buttons. Increment by appropriate value if switch is active.
		chk_buttons();
	}
	isr_timer2_count++;
}

ISR(ADC_vect)
//Adjusts 7 Seg brightness dependent on CDS value 
{
	if(adc_isr_count % 100 == 0)	
		OCR2= ADCH; //only concerned with upper 8. 	
	adc_isr_count++;
}

int8_t debounce_switches(uint8_t i) {
  state[i] = (state[i] << 1) | (! bit_is_clear(PINA, i)) | 0xE000;
  if (state[i] == 0xF000) return 1;
  return 0;
}

void chk_buttons()
{
	uint8_t itr;
	for( itr=0; itr<8; itr++)
	{
		if( debounce_switches(itr))
		{
			switch(itr)
			{
				case 0:
					mode ^= CLOCK_SET;
					break;
				case 1:
					mode ^= ALARM_SET;
					break;
				case 2:
					alarm_set ^= ALARM_ON;
					break;
				case 6:
					snooze ^= SNOOZE_ON;
					break;
				case 7:
					silence_alarm();
					break;
			}
						
		}
	}
}

uint8_t spi_read()
{
	PORTB |= 0x01; //PL is active low, setting it high disables parelel input.
	//PORTE &= ~(1<<6); //clock enable
	SPDR = 0x00; //dummy write
	while(bit_is_clear(SPSR, SPIF)){};
	//PORTE &= (1<<6); // reset clock enable to high 
	return (~SPDR);
}

//                        encoder_direction 
// Determines the encoder directions.
// States:
//	 * 00  |    CCW
//	 * 01  |     ^
//	 * 11  |     |
//	 * 10  V     |
//	 * 00 CW     |
void encoder_direction()
{
	time *ptr;	
	if(mode == CLOCK_SET){ptr = clock_ptr;}
	else if(mode == ALARM_SET){ptr = alarm_ptr;}
 	static uint8_t encoder = 0;
	uint8_t current = 0;
	uint8_t previous = 0;
	encoder = (encoder<<4);
	encoder |= spi_read();
	//right encoder
	current  = ((encoder &  0b00000011));
	previous = ((encoder & 0b00110000) >> 4);
	if((previous == 0b10) && (current == 0b00))
		ptr->min +=1;	
	if((previous == 0b01) && (current == 0b00))
		ptr->min -=1;
	//left encoder	
	current  = ((encoder &  0b00001100) >>2);
	previous = ((encoder & 0b11000000) >> 6);
	if((previous == 0b10) && (current == 0b00))
	 	ptr->min += 1;	
	if((previous == 0b01) && (current == 0b00))
		ptr->min -= 1;
}

void bar_graph(uint8_t display)
{
	SPDR = display;
       	while(bit_is_clear(SPSR, SPIF));
	PORTD = 0x04; //strobe output data reg (regclk)
	PORTD = 0x00;

}

uint8_t itoseven(uint8_t val)
{
    const uint8_t kLookup[10] = 
             {~0x3F, ~0x06, ~0x5B, ~0x4F, ~0x66, 
              ~0x6D, ~0x7D, ~0x07, ~0x7F, ~0x6F };
    return kLookup[val];
                      
}

void rollover()
//Handles the rollover of seconds/min/hr variables
{
	time *ptr;	
	if(mode == CLOCK_SET || mode == DEFAULT){ptr = clock_ptr;}
	else if(mode == ALARM_SET){ptr = alarm_ptr;}
	if(ptr->sec == 60)
	{
		ptr->sec = 0;
		ptr->min +=1;
	}
	if(ptr->min == 60)
	{
		ptr->min = 0;
		ptr->hr += 1;
	}
	if(ptr->min < 0)
	{
		ptr->min = 59;
		ptr->hr -=1;
	}
		
	if(ptr->hr >12)
	{
		ptr->hr = 1;
		//something to toggle PM
	}
	if(ptr->hr < 1)
	{
		ptr->hr =12;
	}
}

void time_disp()
{
  //break up decimal sum into 4 digit-segments
	time t;
	if( (mode == DEFAULT) || (mode == CLOCK_SET)){t = clock;}
	else if(mode == ALARM_SET){t = alarm;}
	else return(0);
	uint8_t ones = t.min% 10;
	uint8_t tens = (t.min/10);
	uint8_t hundreds = t.hr%10;
	uint8_t thousands = (t.hr/10);

	PORTB = DIGIT_ONE;
	PORTA = itoseven(ones);
	_delay_ms(2);
	PORTA = 0xFF;
	PORTB = DIGIT_TWO;
	PORTA = itoseven(tens);
	_delay_ms(2);
	if(colon)
	{
		PORTA = 0xFF; 
		PORTB = DIGIT_COLON;
		PORTA = 0x04;
		_delay_ms(1);
	}
	PORTA = 0xFF;
	PORTB = DIGIT_THREE;
	PORTA = itoseven(hundreds);
	_delay_ms(2);
	PORTA = 0xFF;
	PORTB = DIGIT_FOUR;
	PORTA = itoseven(thousands);
	_delay_ms(2);
}

void check_alarm()
{
	if((clock.hr == alarm.hr) &&(clock.min == alarm.min))
	{
		if (snooze_timer == 0) {TCCR1B |= (1<<CS11) | (1<<CS10);}
		OCR3A = 200;
	}
}
void silence_alarm()
{
	TCCR1B &= ~((1<<CS11) | (1<<CS10));
	OCR3A = 1;
}

int main(void){

	uint16_t lm73_temp;
	char temp_str[2];
	tcnt0_init();
	tcnt1_init();
	tcnt2_init();
	tcnt3_init();

	spi_init();      //initalize SPI 
	lcd_init();      //initalize LCD 
	uart_init();
	init_twi();	//initalize TWI (twi_master.h)  
	cursor_off();

//	adc_init();
	DDRB |= 0xF7; //enable output 4-7
	DDRD |= 0x04;
	spi_init();
	sei();	
	//set LM73 mode for reading temperature by loading pointer register
	//this is done outside of the normal interrupt mode of operation 
	lm73_wr_buf[0] = LM73_PTR_TEMP; //load lm73_wr_buf[0] 
					//with temperature pointer address
	twi_start_wr(LM73_WRITE, lm73_wr_buf, 2); //start the TWI write process
	sei();             //enable interrupts to allow start_wr to finish
	while(1)
	{ 
		DDRA = 0xFF;
		time_disp();

		lcd_lock = 1;
		uart_putc('a');
		lcd_string_array[0] = uart_getc();
		lcd_string_array[1] = uart_getc();
		lcd_lock = 1;
		cursor_home();
		_delay_ms(1);
		char2lcd(lcd_string_array[0]);
		char2lcd(lcd_string_array[1]);
		char2lcd(' ');
		twi_start_rd(LM73_READ, lm73_rd_buf, 2);
		_delay_us(50);
		lm73_temp = (lm73_rd_buf[0]<<8); //save high temperature byte into lm73_temp
		lm73_temp |= lm73_rd_buf[1]; //"OR" in the low temp byte to lm73_temp 
		lm73_temp = (lm73_temp>>7);
		itoa(lm73_temp, temp_str, 10); //convert to string in array with itoa() from avr-libc
		string2lcd(temp_str);
		cursor_home();
		_delay_ms(1);
		lcd_lock = 0;
		if(alarm_set == ALARM_ON)
			check_alarm();
		else {OCR3A = 1;}
		encoder_direction();
		rollover();
	} 
} 

