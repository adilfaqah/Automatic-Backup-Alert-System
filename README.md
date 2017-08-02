# Automatic-Backup-Alert-System
The included device driver interfaces a HC-SR04 Ultrasonic Ranging module and a 8x8 LED Display (that is controlled using a MAX7219 IC) in order to implement a Automatic Back-up Alert System.

The ultrasonic sensor is used to detect the distance between the sensor and an obstacle. This distance is then represented on the LED display as a bar of varying thickness. The greater the thickness of the bar, the greater the distance. If the distance between the sensor and the obstacle is less than a set minimum threshold value, a flashing 'X' is displayed on the LED display.

The driver also supports the following IO control commands:
	- SPI_SET_MIN_THRESH  Set the minimum threshold value/safe distance
	- SPI_SET_MAX_THRESH  Set the maximum threshold value.
	
# Wiring
The wiring of the HC-SR04 Ultrasonic Ranging module is as follows:

VCC -- 5th pin of the Power header (female) on the Galileo board (5V)
TRIG -- IO7 on the Galileo board
ECHO -- IO2 on the Galileo board
GND -- 7th pin of the Power header (female) on the Galileo board (GROUND)


The wiring of the 8x8 LED Display module is as follows:

VCC -- 5V external power supply
GND -- GROUND of external power supply. Remember to make the GROUNDS common through a jumper, i.e. the GROUND of the Galileo board and the GROUND of the external power supply.
DIN -- IO11 on the Galileo board
CS -- IO10 on the Galileo board
CLK -- IO13 on the Galileo board

# Usage
- Cross-compile the module and the user-space test program for the Galileo board using the accompanying Makefile. Transfer the files to the Galileo board.

- Before installing the module remove the existing spidev module by typing and entering: rmmod spidev.ko

- Install the new spidev module by typing and entering: insmod spidev.ko

  >> How to change the min/max threshold values through IOCTL function.
     - Run the user-space test program (to test the IOCTL commands to set the min/max threshold values), type and enter: ./thresh_test

  >> How to change the min/max threshold values through IOCTL function.
     - The min/max threshold values can also be specified as a module parameter. To do that, first ensure that the module is not already installed.
     
	 - Install the module while specifying the module parameters: insmod spidev.ko min_thresh=10 max_thresh=50

NOTE:
	If the user tries to:
	- set the minimum threshold value to less than 2 cm (beyond hardware capability)
	- set the maximum threshold value to greater than 400 cm (beyond hardware capability)
	- set the minimum threshold value to a value greater than the maximum threshold value
	- set the maximum threshold value to a value lesser than the minimum threshold value
	
	through the IOCTL function, an error will be returned.
	
	If the user tries to do the same by specifying the module parameters while installing the module, the driver will revert to default values (10 for minimum threshold and 100 for maximum threshold)
