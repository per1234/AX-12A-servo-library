/*
 * Example showing how to reag internal register of AX-12A
 * Can be usefull for debugging purposes
 */

#include "DynamixelSerial.h"
#include "Arduino.h"

unsigned char Direction_Pin = 10;
long baud = 1000000;
unsigned int ID = 1;

int reg = 0;

void setup()
{
	Dynamixel.begin(baud, Direction_Pin, &Serial);
}

void loop()
{
	reg = Dynamixel.readRegister(ID, AX_PRESENT_VOLTAGE, 1);
	Serial.println(reg);

	delay(1000);
}

