/*
 * Example showing how to read internal register of AX-12A
 * Can be usefull for debugging purposes
 */

#include "Arduino.h"
#include "../../src/AX12A.h"

unsigned char Direction_Pin = 10;
long baud = 1000000;
unsigned int ID = 1;

int reg = 0;

void setup()
{
	AX12A.begin(baud, Direction_Pin, &Serial);
}

void loop()
{
	reg = AX12A.readRegister(ID, AX_PRESENT_VOLTAGE, 1);
	Serial.println(reg);

	delay(1000);
}

