/*
 * Example showing how to send position commands to AX-12A
 */

#include "Arduino.h"
#include "../../src/AX12A.h"

unsigned char Direction_Pin = 10;
long baud = 1000000;
unsigned int ID = 1;

int initial_pos = 512;
int max = initial_pos + 100;
int min = initial_pos - 100;

int pos = initial_pos;
int delta = 5;

void setup()
{
	AX12A.begin(baud, Direction_Pin, &Serial);
}

void loop()
{
	pos = pos + delta;

	if (pos > max)
	{
		pos = max;
		delta *= -1;
	}

	if (pos < min)
	{
		pos = min;
		delta *= -1;
	}

	AX12A.move(ID, pos);
	delay(20);
}
