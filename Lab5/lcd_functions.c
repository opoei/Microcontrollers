//
//misc LCD functions for mega128 board
//refactored by R. Traylor 10.7.09

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>

#include "lcd_functions.h"

char lcd_str[16];  //holds string to send to lcd  

void strobe_lcd(void){
	//twiddles bit 3, PORTF creating the enable signal for the LCD
	PORTF |= 0x08;
	PORTF &= ~0x08;
}          
 
void clear_display(void){
	SPDR = 0x00;    //command, not data
	while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
	SPDR = 0x01;    //clear display command
	while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
	strobe_lcd();   //strobe the LCD enable pin
	_delay_ms(2);   //obligatory waiting for slow LCD
}         

void cursor_home(void){
	SPDR = 0x00;    //command, not data
	while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
	SPDR = 0x02;   // cursor go home position
	while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
	strobe_lcd();
//	_delay_ms(1);  //not necessary with alarm_clock.c code
}         
  
void home_line2(void){
	SPDR = 0x00;    //command, not data
	while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
	SPDR = 0xC0;   // cursor go home on line 2
	while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
	strobe_lcd(); 
//	_delay_ms(1);    //not necessary with alarm_clock.c code
}                           
 
void fill_spaces(void){
	int count;
	for (count=0; count<=15; count++){
		SPDR = 0x01; //set SR for data
		while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
		SPDR = 0x20; 
		while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
		strobe_lcd();
		_delay_ms(1);
	}
}  
   
void char2lcd(char a_char){
	//sends a char to the LCD
	//usage: char2lcd('H');  // send an H to the LCD
	SPDR = 0x01;   //set SR for data xfer with LSB=1
	while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
	SPDR = a_char; //send the char to the SPI port
	while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
	strobe_lcd();  //toggle the enable bit
	_delay_ms(1); //wait the prescribed time for the LCD to process
}
  
void cursor_off(void){
	//cuts cursor off 
	SPDR = 0x00;   //set SR for data xfer with LSB=1
	while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
	SPDR = 0x0C; //send the command to cut off cursor
	while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
	strobe_lcd();  //toggle the enable bit
	_delay_ms(1); //wait the prescribed time for the LCD to process
}
  
void string2lcd(char *lcd_str){

	//sends a string to LCD
	int count;
	for (count=0; count<=(strlen(lcd_str)-1); count++){
		SPDR = 0x01; //set SR for data
		while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
		SPDR = lcd_str[count]; 
		while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
		strobe_lcd();
		_delay_us(100);
	}                  
} 

void lcd_init(void){
	int i;
	DDRF |= 0x08;  //port F bit 3 is the enable strobe for the LCD
	//initalize the LCD to receive data
	_delay_ms(15);   
	for(i=0; i<=2; i++){ //do funky initalize sequence 3 times
		SPDR = 0x00;
		while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
		SPDR = 0x30;
		while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
		strobe_lcd();
		_delay_ms(7);
	}

	SPDR = 0x00;
	while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
	SPDR = 0x38;
	while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
	strobe_lcd();
	_delay_ms(5);   

	SPDR = 0x00;
	while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
	SPDR = 0x08;
	while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
	strobe_lcd();
	_delay_ms(5);

	SPDR = 0x00;
	while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
	SPDR = 0x01;
	while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
	strobe_lcd();
	_delay_ms(5);   

	SPDR = 0x00;
	while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
	SPDR = 0x06;
	while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
	strobe_lcd();
	_delay_ms(5);

	SPDR = 0x00;
	while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
	SPDR = 0x0E;
	while (!(SPSR & 0x80)) {}	// Wait for SPI transfer to complete
	strobe_lcd();
	_delay_ms(5);
} 

