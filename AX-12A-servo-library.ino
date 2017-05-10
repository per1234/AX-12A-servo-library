#include "Arduino.h"
#include "src/AX12A.h"

#define DirectionPin 	(10u)
#define BaudRate  		(1000000ul)
#define ID				(1u)

void setup()
{
	AX12A.begin(BaudRate, DirectionPin, &Serial);
}

void loop()
{
	AX12A.ledStatus(ID, ON);
	delay(1000);
	AX12A.ledStatus(ID, OFF);
	delay(1000);
}
