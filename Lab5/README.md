#Lab4
- Builds on all previous labs.
- Adds I2C communication with temp sensor, and remote temp sensor on atmega48 board.
TODO: Lots of stuff. Most the code works on its own, however integration was rough. Getting all the timing just right is the primary issue. May want to put the rs232 on a logic analyzer/oscope. People reported bad communication via the RS232 connector. 
##atmega48
- Contains all files needed for I2C communication, setup as slave.
##twi
- I2C libraries
##lcd_functions
- Libraries for communicating with LCD board
