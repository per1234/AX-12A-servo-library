#include "DynamixelSerial.h"
#include "Arduino.h"

unsigned char Direction_Pin = 10;
long baud = 1000000;

void setup()
{
	Dynamixel.begin(baud, Direction_Pin, &Serial);
	Dynamixel.setEndless(1,ON);
	Dynamixel.turn(1,LEFT,100);
}

void loop()
{

}
