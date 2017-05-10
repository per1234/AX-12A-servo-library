/*
 * Example showing how to use endless mode (wheel mode) on AX-12A
 * Be sure you removed all mechanical assemblies (hinges) before using this code !
 */
#include "Arduino.h"
#include "../../src/AX12A.h"

unsigned char Direction_Pin = 10;
long baud = 1000000;
unsigned int ID = 1;

void setup()
{
	AX12A.begin(baud, Direction_Pin, &Serial);
	AX12A.setEndless(ID, ON);
	AX12A.turn(ID, LEFT, 100);
}

void loop()
{

}
