/*
 * Example showing how to use endless mode (wheel mode) on AX-12A
 * Be sure you removed all mechanical assemblies (hinges) before using this code !
 */
#include "Arduino.h"
#include "../../src/AX12A.h"

#define DirectionPin 	(10u)
#define BaudRate  		(1000000ul)
#define ID				(1u)

void setup()
{
	AX12A.begin(BaudRate, DirectionPin, &Serial);
	AX12A.setEndless(ID, ON);
	AX12A.turn(ID, LEFT, 100);
}

void loop()
{

}
