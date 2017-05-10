/*
 * Example of a simple Blink using the AX-12A built-in LED
 */

#include "Arduino.h"
#include "../../src/AX12A.h"

unsigned char Direction_Pin = 10;
long baud = 1000000;
unsigned int ID = 1;

void setup()
{
	AX12A.begin(baud, Direction_Pin, &Serial);
}

void loop()
{
	AX12A.ledStatus(ID, ON);
	delay(1000);
	AX12A.ledStatus(ID, OFF);
	delay(1000);
}
