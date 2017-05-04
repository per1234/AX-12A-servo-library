/*
 * Example showing how to use endless mode (wheel mode) on AX-12A
 * Be sure you removed all mechanical assemblies (hinges) before using this code !
 */
#include "DynamixelSerial.h"
#include "Arduino.h"

unsigned char Direction_Pin = 10;
long baud = 1000000;
unsigned int ID = 1;

void setup()
{
	Dynamixel.begin(baud, Direction_Pin, &Serial);
	Dynamixel.setEndless(ID, ON);
	Dynamixel.turn(ID, LEFT, 100);
}

void loop()
{

}
