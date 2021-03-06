// ***************************************************************************************
// L293DMasterLcdRotary.ino - Example Arduino Master program controlling multiple
//                            Slaves with L293D Motor Driver boards and solenoids.
//                            LCD and Rotary Encoder allow one to select which
//                            Solenoid to fire.
//
//    Author:         John Miller
//    Revision:       1.0.0
//    Date:           3/17/2018
//    Project Source: https://github.com/jpsrmiller/l293d-master-slave
// 
// This Arduino program used a KY-040 Rotary Encoder and
//   a 16x2 or 20x4 Character LCD with I2C interface
// 
// The program may be freely copied, modified, and used for any Arduino projects
//
// Slave Arduinos are connected to Pins 8 and 9
// See https://buildmusic.net/tutorials/motor-driver/ for a wiring schematic
//
// The KY-040 Rotary Encoder is connected to the following Arduino Uno pins
//      o CLK --> Pin 2
//      o DT  --> Pin 3
//      o SW  --> Pin 4
//      o +   --> Pin 5
//      o GND --> Pin 6
// 
// The LCD is connected to the following Arduino Uno pins
//      o GND --> GND
//      o VCC --> 5V
//      o SDA --> Pin A4 (SDA)
//      o SCL --> Pin A5 (SCL)
//
// The Rotary Encoder uses Arduino pins for the GND and +5V so that the remaining
//   5V and GND pins on the Arduino Uno can be used for other peripheral devices
// This works because the the Rotary Encoder draws less than the 40 mA
//   maximum current allowed on the Arduino Uno I/O pins  
//
// *** External Libraries Required ***
// The following libraries must be downloaded and copied into the Arduino "libraries"
// folder in order for the program to compile:
//    o OneButton - https://github.com/mathertel/OneButton
//    o LiquidCrystal_I2C - https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
//
// ***********************************************************************************

#include <Arduino.h>
#include <Wire.h>
#include <OneButton.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// Define the IO Pins Used
#define PIN_ROTARY_CLK    2   // Used for generating interrupts using CLK signal
#define PIN_ROTARY_DAT    3   // Used for reading DT signal
#define PIN_ROTARY_SW     4   // Used for the Rotary push button switch
#define PIN_ROTARY_5V     5   // Set to HIGH to be the 5V pin for the Rotary Encoder
#define PIN_ROTARY_GND    6   // Set to LOW to be the GND pin for the Rotary Encoder
#define PIN_SW_SERIAL_RX  8   // Software Serial Recieve pin
#define PIN_SW_SERIAL_TX	9   // Software Serial Transmit pin

// Most I2C LCD's have an I2C Address of either 0x27 or 0x3F
// If the LCD doesn't work with one address, try the other
#define LCD_I2C_ADDRESS 0x27
//#define LCD_I2C_ADDRESS     0x3F

// Define the size of the LCD.  Most LCD's are either 16x2 or 20x4
#define LCD_ROW_COUNT       4    // Number of Rows
#define LCD_COL_COUNT       20   // Number of Characters per Row

// Define the total number of Slave Arduino's with Motor Shields connected
#define MOTOR_SHIELD_COUNT	4

// Total number of Channels 
int totalChannels = 8 * MOTOR_SHIELD_COUNT;

// Selected Shield and Channel, based on rotaryCount
uint8_t selectedShield;
uint8_t selectedChannel;

// OneButton class handles Debounce and detects button press
OneButton btnRot(PIN_ROTARY_SW, HIGH);		  // Rotary Select button

LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COL_COUNT, LCD_ROW_COUNT);

SoftwareSerial swSerial(PIN_SW_SERIAL_RX, PIN_SW_SERIAL_TX);

// Used for the Rotary Encoder interrupt routines PinA() and PinB()
volatile int rotaryCount = 0;

// Disables the Rotary Encoder interrupts while the LCD is being updated
byte rotaryDisabled;

volatile byte aFlag = 0; // lets us know when we're expecting a rising edge on pinA 
						 // to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // lets us know when we're expecting a rising edge on pinB 
						 // to signal that the encoder has arrived at a detent 
						 // (opposite direction to when aFlag is set)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt 
						   // pins before checking to see if we have moved a whole detent

// ****************************************************************************
// PinA() - Called by the Interrupt pin when the Rotary Encoder Turned
//    Routine taken from:  
//    https://exploreembedded.com/wiki/Interactive_Menus_for_your_project_with_a_Display_and_an_Encoder
// ****************************************************************************
void PinA() {

	if (rotaryDisabled) return;

	cli(); //stop interrupts happening before we read pin values
		   // read all eight pin values then strip away all but pinA and pinB's values
	reading = PIND & 0xC;

	//check that both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
	if (reading == B00001100 && aFlag) {
		rotaryUp();
		bFlag = 0; //reset flags for the next turn
		aFlag = 0; //reset flags for the next turn
	}
	//signal that we're expecting pinB to signal the transition to detent from free rotation
	else if (reading == B00000100) bFlag = 1;
	sei(); //restart interrupts
}

// ****************************************************************************
// PinB() - Called by the Interrupt pin when the Rotary Encoder Turned
//    Routine taken from:  
//    https://exploreembedded.com/wiki/Interactive_Menus_for_your_project_with_a_Display_and_an_Encoder
// ****************************************************************************
void PinB() {

	if (rotaryDisabled) return;

	cli(); //stop interrupts happening before we read pin values
		   //read all eight pin values then strip away all but pinA and pinB's values
	reading = PIND & 0xC;
	//check that both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge 
	if (reading == B00001100 && bFlag) {
		rotaryDown();
		bFlag = 0; //reset flags for the next turn
		aFlag = 0; //reset flags for the next turn
	}
	//signal that we're expecting pinA to signal the transition to detent from free rotation
	else if (reading == B00001000) aFlag = 1;
	sei(); //restart interrupts
}

// ****************************************************************************
// rotaryUp() - Rotary Encoder is turned 1 detent to the Right (clockwise)
// **************************************************************************** 
void rotaryUp()
{
	rotaryCount++;
	if (rotaryCount > totalChannels-1) rotaryCount = totalChannels - 1;
}

// **********************************************************************************
// rotaryDown() - Rotary Encoder is turned 1 detent to the Left (counter-clockwise) 
// **********************************************************************************
void rotaryDown()
{
	rotaryCount--;
	if (rotaryCount < 0) rotaryCount = 0;	
}

// ****************************************************************************
// rotaryClick() - Rotary Encoder Select Switch is pressed
// ****************************************************************************
void rotaryClick()
{
	testNote(selectedShield, selectedChannel);
}

// ****************************************************************************
// rotaryLongPress() - Rotary Encoder Select Switch is Held Down (Long Press)
// ****************************************************************************
void rotaryLongPress()
{
	rotaryCount = 0;
}

// ****************************************************************************
// initializeRotaryEncoder() - Initialize the pins and interrupt functions
//                             for the Rotary Encoder
// ****************************************************************************
void initializeRotaryEncoder()
{
	// Set the Directions of the I/O Pins
	pinMode(PIN_ROTARY_CLK, INPUT_PULLUP);
	pinMode(PIN_ROTARY_DAT, INPUT_PULLUP);
	pinMode(PIN_ROTARY_SW, INPUT_PULLUP);
	pinMode(PIN_ROTARY_GND, OUTPUT);
	pinMode(PIN_ROTARY_5V, OUTPUT);

	// Set the 5V and GND pins for the Rotary Encoder
	digitalWrite(PIN_ROTARY_GND, LOW);
	digitalWrite(PIN_ROTARY_5V, HIGH);

	// set an interrupt on PinA and PinB, looking for a rising edge signal and 
	// executing the "PinA" and "PinB" Interrupt Service Routines
	attachInterrupt(0, PinA, RISING);
	attachInterrupt(1, PinB, RISING);

	// Define the functions for Rotary Encoder Click and Long Press
	btnRot.attachClick(&rotaryClick);
	btnRot.attachLongPressStart(&rotaryLongPress);
	btnRot.setPressTicks(2000);

	rotaryDisabled = 0;
}

// ****************************************************************************
// initializeLcdMotorShield() - Initialize the LCD
// ****************************************************************************
void initializeLcdMotorShield()
{
	lcd.begin();
	lcd.backlight();
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(F("Test Motor Shields"));
	lcd.setCursor(0, 1);
	lcd.print(F("-------------------"));
}

// ****************************************************************************
// updateLcdMotorShield() - Update the LCD with Slave Address and Channel
// ****************************************************************************
void updateLcdMotorShield()
{
	rotaryDisabled = 1;
	selectedChannel = rotaryCount % 8;
	selectedShield = (rotaryCount - selectedChannel) / 8;

	lcd.setCursor(0, 2);
	lcd.print(F("Slave Address = "));
	lcd.print(selectedShield);
	lcd.print(F("   "));
	lcd.setCursor(0, 3);
	lcd.print(F("Channel = "));
	lcd.print(selectedChannel);
	lcd.print(F("   "));
	rotaryDisabled = 0;
}

// ****************************************************************************
// setup() - Initialization Function
// ****************************************************************************
void setup()
{
	initializeRotaryEncoder();
	swSerial.begin(57600);
	initializeLcdMotorShield();
}

// ****************************************************************************
// loop() - Main Program Loop Function 
// ****************************************************************************
void loop()
{
	updateLcdMotorShield();
	btnRot.tick();
	delay(100);
}

// ****************************************************************************
// testNote() - energize channel multiple times to play a note 
// ****************************************************************************
void testNote(uint8_t slaveAddr, uint8_t channelIndex)
{
	uint8_t i;

	for (i = 0; i < 3; i++)
		playNote(slaveAddr, channelIndex);
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
