#include "Arduino.h"
#include "src/AX12A.h"

unsigned char Direction_Pin = 10;
long baud = 1000000;

void setup()
{
	AX12A.begin(baud, Direction_Pin, &Serial);
}

void loop()
{
	AX12A.ledStatus(1, ON);
	delay(1000);
	AX12A.ledStatus(1, OFF);
	delay(1000);
}
