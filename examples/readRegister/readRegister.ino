/*
 * Example showing how to read internal register of AX-12A
 * Can be usefull for debugging purposes
 */

#include "Arduino.h"
#include "../../src/AX12A.h"

#define DirectionPin 	(10u)
#define BaudRate  		(1000000ul)
#define ID				(1u)

int reg = 0;

void setup()
{
	AX12A.begin(BaudRate, DirectionPin, &Serial);
}

void loop()
{
	reg = AX12A.readRegister(ID, AX_PRESENT_VOLTAGE, 1);
	Serial.println(reg);

	delay(1000);
}

