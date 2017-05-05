/*
 * Scan and test usable baud rates by your microcontroler
 */

#include "DynamixelSerial.h"
#include "Arduino.h"

unsigned char Direction_Pin = 10;
unsigned int ID = 1;

int i = 0;
int j = 0;

int volt = 0;
int reg = 0;

unsigned const int nb_test = 18;

long bdRates[nb_test] = {
							1000000, 	1, // 0 , 1
							500000, 	3, // 2 , 3
							400000,		4, // 4 , 5
							250000,		7,
							200000,		9,
							115200,		16,
							57600,		34,
							19200,		103,
							9600,		207
						};

void setup()
{
	SerialUSB.begin(9600);

	for (j = 0; j < nb_test; j = j + 2) // reset all
	{
		Dynamixel.begin(bdRates[j], Direction_Pin, &Serial1);
		delay(100);
		SerialUSB.print("reset ");SerialUSB.print(bdRates[j]);SerialUSB.println(" done.");
		Dynamixel.reset(ID);
		delay(100);
		Dynamixel.end();
		delay(100);
	}
}

void loop()
{
	delay(2000);
	SerialUSB.println("*** Start test ***");
	Dynamixel.begin(bdRates[i], Direction_Pin, &Serial1);
	delay(2000);

	int reg = Dynamixel.readRegister(ID, AX_BAUD_RATE, 1);
	SerialUSB.print("AX_BAUD_RATE = ");SerialUSB.println(reg);
	delay(2000);

	SerialUSB.print("Baud Rate set to ");SerialUSB.println(bdRates[i]);
	delay(2000);

	do
	{
		volt = Dynamixel.readVoltage(ID);
		SerialUSB.print("volt = ");SerialUSB.println(volt);
		delay(500);
	} while (volt != 117);

	delay(2000);

	if (i == 16 )
	{
		Dynamixel.setBD(ID,bdRates[0]);
		SerialUSB.print("Baud Rate set to ");SerialUSB.print(bdRates[0]);SerialUSB.println(" for the next test");
	}
	else
	{
		Dynamixel.setBD(ID,bdRates[i + 2]);
		SerialUSB.print("Baud Rate set to ");SerialUSB.print(bdRates[i + 2]);SerialUSB.println(" for the next test");
	}

	delay(2000);

	Dynamixel.end();

	SerialUSB.println("*** END test ***");
	SerialUSB.println(" ");

	i = i + 2;

	if (i == nb_test)
		{
			i = 0;
		}
	delay(1000);
}
