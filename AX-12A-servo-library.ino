#include "Arduino.h"
#include "src/AX12A.h"

#define DirectionPin 	(2u)
#define BaudRate  		(1000000ul)
#define ID				(1u)

void setup()
{
	AX12A.begin(BaudRate, DirectionPin, &Serial1);
}

void loop()
{
	AX12A.ledStatus(1, ON);
	delay(1000);
	AX12A.ledStatus(1, OFF);
	delay(1000);
}
