/*
 * Example of a simple Blink using the AX-12A built-in LED
 */

#include "DynamixelSerial.h"
#include "Arduino.h"

unsigned char Direction_Pin = 10;
long baud = 1000000;

void setup()
{
	Dynamixel.begin(baud, Direction_Pin, &Serial);
}

void loop()
{
	Dynamixel.ledStatus(1, ON);
	delay(1000);
	Dynamixel.ledStatus(1, OFF);
	delay(1000);
}
