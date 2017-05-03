#include <DynamixelSerial.h>

#define sendData(args)  (varSerial->write(args))    // Write Over Serial
#define availableData() (varSerial->available())    // Check Serial Data Available
#define readData()      (varSerial->read())         // Read Serial Data
#define peekData()      (varSerial->peek())         // Peek Serial Data
#define beginCom(args)  (varSerial->begin(args))    // Begin Serial Comunication 
#define endCom()        (varSerial->end())          // End Serial Comunication

#define setDPin(DirPin,Mode)   (pinMode(DirPin,Mode))       // Select the Switch to TX/RX Mode Pin
#define switchCom(DirPin,Mode) (digitalWrite(DirPin,Mode))  // Switch to TX/RX Mode

#define delayus(args) (delayMicroseconds(args))  // Delay Microseconds

unsigned char Direction_Pin = 2;
long baud = 500000;

void setup()
{
	//Dynamixel.begin(baud, Direction_Pin, &Serial1);
	Serial.begin(baud);
	setDPin(Direction_Pin, OUTPUT);
}	

void loop()
{
	//Dynamixel.ledStatus(1, ON);
	sendLed(1, ON);
	digitalWrite(LED_BUILTIN, HIGH);
	delay(1500);
	
	//Dynamixel.ledStatus(1, OFF);
	sendLed(1, OFF);
	digitalWrite(LED_BUILTIN, LOW);
	delay(1500);

	/*Serial.println("coucou");
	delay(1500);*/
}

void sendLed(unsigned char ID, char Status)
{
	const unsigned int length = 8;
	byte packet[length];

	byte Checksum = (~(ID + AX_LED_LENGTH + AX_WRITE_DATA + AX_LED + Status)) & 0xFF;

	packet[0] = AX_START;
	packet[1] = AX_START;
	packet[2] = ID;
	packet[3] = AX_LED_LENGTH;
	packet[4] = AX_WRITE_DATA;
	packet[5] = AX_LED;
	packet[6] = Status;
	packet[7] = Checksum;

	switchCom(Direction_Pin, Tx_MODE);

	Serial.write(packet, length);
	
	Serial.flush();

	//delayus(10);

	switchCom(Direction_Pin, Rx_MODE);
}