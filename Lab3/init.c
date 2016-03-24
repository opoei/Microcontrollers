
#include <avr/io.h>
#include  <avr/interrupt.h>



/***********************************************************************/
//                            spi_init                               
//Initalizes the SPI port on the mega128. Does not do any further   
//external device specific initalizations.  Sets up SPI to be:                        
//master mode, clock=clk/2, cycle half phase, low polarity, MSB first
//interrupts disabled, poll SPIF bit in SPSR to check xmit completion
/***********************************************************************/
void spi_init(void)
{
  DDRB  |=   0x07;		//Turn on SS, MOSI, SCLK
  SPCR  |=   ( 1<<SPE | 1<<MSTR );           //set up SPI mode
  SPSR  |=   (1<<SPI2X); 	// double speed operation
 }

/***********************************************************************/
//                              tcnt0_init                             
//Initalizes timer/counter0 (TCNT0). TCNT0 is running in async mode
//with external 32khz crystal.  Runs in normal mode with no prescaling.
//Interrupt occurs at overflow 0xFF.
/***********************************************************************/
void tcnt0_init(void)
{
  ASSR   |=  (1<<AS0);  //ext osc TOSC
  TIMSK  |=  (1<<TOIE0);  //enable timer/counter0 overflow interrupt
  TCCR0  |=  (1<<CS00);  //normal mode, no prescale
}
