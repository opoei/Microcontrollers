#Lab5
- Builds on all previous labs.
- Adds I2C communication with temp sensor, and remote temp sensor on atmega48 board.

TODO: Lots of stuff. 
- Most the code works on its own, however integration was rough. Getting all the timing just right is the primary issue. 
- May want to put the rs232 interface on a logic analyzer/oscope. People reported bad communication via the RS232 connector. 
- I2C_POC.zip contains the proof of concept that i2c works and is set up correctly. When flashed should report temperature data for both local and remote temp sensors via i2c, and print out on the LCD

###atmega48
- Contains all files needed for I2C communication from the atmega48 board setup as slave.

###twi
- I2C libraries as provided

###lcd_functions
- Libraries for communicating with LCD board as provided
