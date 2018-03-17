// ***************************************************************************************
// L293DMasterTest.ino - Example Arduino Master program controlling multiple
//                       Slaves with L293D Motor Driver boards and solenoids.
//
//    Author:         John Miller
//    Revision:       1.0.0
//    Date:           3/17/2018
//    Project Source: https://github.com/jpsrmiller/l293d-master-slave
// 
// The program may be freely copied, modified, and used for any Arduino projects
//
// Slave Arduinos are connected to Pins 8 and 9
// See https://buildmusic.net/tutorials/motor-driver/ for a wiring schematic
//
// ***********************************************************************************

#include <Arduino.h>
#include <SoftwareSerial.h>

// Define the IO Pins Used
#define PIN_SW_SERIAL_RX  8   // Software Serial Recieve pin
#define PIN_SW_SERIAL_TX	9   // Software Serial Transmit pin

// Define the total number of Slave Arduino's with Motor Shields connected
#define MOTOR_SHIELD_COUNT	4

SoftwareSerial swSerial(PIN_SW_SERIAL_RX, PIN_SW_SERIAL_TX);

// ****************************************************************************
// setup() - Initialization Function
// ****************************************************************************
void setup()
{
	swSerial.begin(57600);
}

// ****************************************************************************
// loop() - Main Program Loop Function 
// ****************************************************************************
void loop()
{
	delay(2000);
	serialSyntaxExample();
	delay(2000);
	oneSolenoidExample();
	delay(2000);
	threeSolenoidExample();
	delay(2000);
	multiSlaveExample();
	delay(2000);
	energizeSolenoidsInOrder();
	delay(2000);
}

// ****************************************************************************
// serialSyntaxExample() - Example of printing messages directly to Serial 
// ****************************************************************************
void serialSyntaxExample()
{
	// Energize Slave 0 Channels 0 and 1
	swSerial.print(F("<0003>"));
	delay(100);
	//De-energize All Channels
	swSerial.print(F("<0000>"));
	delay(1000);

	// Energize Slave 0 Channel 1, Slave 1 Channel 3, Slave 2 Channel 5 and Slave 3, Channel 7
	swSerial.print(F("<0002><0108><0220><0380>"));
	delay(100);
	//De-energize All Channels
	swSerial.print(F("<0000><0100><0200><0300>"));
	delay(1000);
}

// ****************************************************************************
// oneSolenoidExample() - Example of energizing one solenoid at a time 
// ****************************************************************************
void oneSolenoidExample()
{
	// Energize Slave 0 Channel 4
	sendMotorShieldCommand(0, B00010000);
	delay(100);
	sendMotorShieldCommand(0, 0); // De-energize all channels
	delay(1000);

	// Energize Slave 1 Channel 2
	sendMotorShieldCommand(1, B00000100);
	delay(100);
	sendMotorShieldCommand(1, 0); // De-energize all channels
	delay(1000);

	// Energize Slave 2 Channel 7
	sendMotorShieldCommand(2, B10000000);
	delay(100);
	sendMotorShieldCommand(2, 0); // De-energize all channels
	delay(1000);

	// Energize Slave 3 Channel 5
	sendMotorShieldCommand(3, B00100000);
	delay(100);
	sendMotorShieldCommand(3, 0); // De-energize all channels
	delay(1000);
}

// ****************************************************************************
// threeSolenoidExample() - Example of energizing three solenoids at once 
// ****************************************************************************
void threeSolenoidExample()
{
	// Energize Slave 0 Channels 1, 3, and 5
	sendMotorShieldCommand(0, B00101010);
	delay(100);
	sendMotorShieldCommand(0, 0); // De-energize all channels
	delay(1000);

	// Energize Slave 1 Channels 0, 1, and 2
	sendMotorShieldCommand(1, B00000111);
	delay(100);
	sendMotorShieldCommand(1, 0); // De-energize all channels
	delay(1000);
}

// *********************************************************************************
// multiSlaveExample() - Example of energizing solenoids in multiple slaves at once 
// *********************************************************************************
void multiSlaveExample()
{
	// Energize Slave 0 Channels 0 and 1
	sendMotorShieldCommand(0, B00000011);
	// Energize Slave 1 Channels 3 and 4
	sendMotorShieldCommand(1, B00011000);
	// Energize Slave 2 Channels 5 and 7
	sendMotorShieldCommand(2, B10100000);
	delay(100);
	sendMotorShieldCommand(0, 0); // De-energize all channels
	sendMotorShieldCommand(1, 0); // De-energize all channels
	sendMotorShieldCommand(2, 0); // De-energize all channels
	delay(100);
}

// *********************************************************************************
// energizeSolenoidsInOrder() - Energize all solenoids sequentially 
// *********************************************************************************
void energizeSolenoidsInOrder()
{
	uint8_t i, j;
	for (i = 0; i<MOTOR_SHIELD_COUNT; i++)
	{
		for (j = 0; j<8; j++)
			playNote(i, j);
	}
}


// ****************************************************************************
// playNote() - energize and de-energize a channel once 
// ****************************************************************************
void playNote(uint8_t slaveAddr, uint8_t channelIndex)
{

	sendMotorShieldCommand(slaveAddr, 1 << channelIndex);
	delay(80);
	sendMotorShieldCommand(slaveAddr, 0);
	delay(500);
}

// ****************************************************************************
// sendMotorShieldCommand() - Send a Serial Command to the Slave
//       Serial Command is of the form: <aabb>
//       where aa is the Slave Address in Hexadecimal
//       and   bb is a Bitmask of the channels to be energized
// ****************************************************************************
void sendMotorShieldCommand(uint8_t boardNum, uint8_t pdata)
{
	swSerial.print(F("<"));
	if (boardNum < 0x10)
		swSerial.print(F("0"));
	swSerial.print(boardNum, HEX);
	if (pdata < 0x10)
		swSerial.print(F("0"));
	swSerial.print(pdata, HEX);
	swSerial.println(F(">"));
	return;
}