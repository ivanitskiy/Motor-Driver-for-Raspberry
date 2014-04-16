Motor-Driver-for-Raspberry
==========================

This allows you to control motor-driver shield for Arduion connected to your Raspberry Pi

Connect your Pi to driver shild:
  pink  GPIO 4  -> 11 motor_1
  blue  GPIO 3  -> 8  SER - serial input, hooks up to the serial output of your micro controller
  green GPIO 2  -> 4  SRCLK - shift register clock (positive edge triggered); it times the input to SER
  gray  GPIO 17 -> 12 RCLK - storage register clock (positive edge triggered); it is used to latch the register to the outputs
  brown GPIO 27 -> 7  OE - output enable


Tets:

Tested with one dc motor conneted to the shield. Stop, reverse, forward works fine.


Useful links:
 - https://pypi.python.org/pypi/RPi.GPIO
