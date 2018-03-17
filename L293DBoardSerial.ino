// *************************************************************************************
// L293DBoardSerial.ino - Program for Arduino Slave controlling up to 8 Solenoids
//                        via the L293D Motor Driver Shield
//    Author:         John Miller
//    Revision:       1.0.0
//    Date:           3/17/2018
//    Project Source: https://github.com/jpsrmiller/l293d-master-slave
// 
// The program recieves messages from an Arduino Master via serial.
// If the device address in the message matches the address of the slave, then
//    the program energizes solenoids per the message.
// After processing the message, the program transmits the same message via
//    serial, so that the message can be seen by the next Slave in the line
//
// The serial message needs to be in the format:
//               <aabb>
//  where aa and bb are bytes in Hexadecimal format.
//         aa is the device address
//         bb is a bitmask corresponding to the channel(s) to be energized
//
// Each solenoid is wired between GND and one of the Motor Terminals:
//    M1A = Channel 0
//    M1B = Channel 1
//    M2A = Channel 2
//    M2B = Channel 3
//    M3A = Channel 4
//    M3B = Channel 5
//    M4A = Channel 6
//    M4B = Channel 7
//
// For example, to energize channels 2, 4, and 7 on Slave 1, the Master sends
//    the command:     <0194>   (94 in Hex = 10010100 in Binary)
//
// See the project web-site for pictures and more information:
//            https://buildmusic.net/tutorials/motor-driver/
//
// Code for controlling the Motor Shield is taken from the example on the Arduino website:
//   https://playground.arduino.cc/Main/AdafruitMotorShield
//
// ************************************************************************************


// *********************************************************************************
// The following #define constants may need to be changed per instructions
// *********************************************************************************

// *********************************************************************************
// Set to 1 to test hardware by having the Arduino energize the solenoids in order
// Set to 0 for normal operation (energizing channels in response to Serial commands)
// See:  https://buildmusic.net/tutorials/motor-driver/
#define HARDWARE_TEST	0
// *********************************************************************************

// *********************************************************************************
// The Address of the Slave is defined here.  This is used for the
//    Master to identify the correct Slave to control
// See: https://buildmusic.net/assembly/electronics/
#define DEVICE_ADDRESS	0
// *********************************************************************************



// *********************************************************************************
// The remainder of the code should typically not need to be changed
// *********************************************************************************

// Arduino pins for the shift register
#define MOTORLATCH   12
#define MOTORCLK     4
#define MOTORENABLE  7
#define MOTORDATA    8

// 8-bit bus after the 74HC595 shift register 
// (not Arduino pins)
// These are used to set the direction of the bridge driver.
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR3_A 5
#define MOTOR3_B 7
#define MOTOR4_A 0
#define MOTOR4_B 6

// Arduino pins for the PWM signals.
#define MOTOR1_PWM 11
#define MOTOR2_PWM 3
#define MOTOR3_PWM 6
#define MOTOR4_PWM 5

// Various defines related to Serial messages
#define SERIAL_MAX_DATA_LENGTH		32
#define SERIAL_MSG_BEGIN_CHAR		'<'
#define SERIAL_MSG_END_CHAR			'>'
#define SERIAL_BAUD_RATE			57600


// Array storing the 8-bit bus after the 74HC595 shift register
uint8_t motorBus[] = { MOTOR1_A , MOTOR1_B, MOTOR2_A , MOTOR2_B, 
                       MOTOR3_A , MOTOR3_B, MOTOR4_A , MOTOR4_B };

// Array storing the Arduino pins for the PWM signals
uint8_t motorPwmPins[] = { MOTOR1_PWM, MOTOR2_PWM, MOTOR3_PWM, MOTOR4_PWM };


uint8_t serialData[SERIAL_MAX_DATA_LENGTH]; // Buffer to recieve the Serial Messages
uint8_t deviceAddress; // Address of the Slave

// Used for processing serial messages
uint8_t serialMessageActive;
uint8_t serialMessageLength;
uint8_t serialNextChar;
uint8_t serialMessageAvailable;

// Used for Hardware Test mode (when HARDWARE_TEST = 1)
uint8_t testHardwareSolenoidCount;
uint8_t testHardwarePauseCount;

// Stores last serial communication time and last Channel set time
// After a time-out period any serial messages in progress are discarded
//   and any energized channels are de-energized
unsigned long serialLastTime;
unsigned long setChannelsLastTime;

// ****************************************************************************
// setup() - Initialization Function
// ****************************************************************************
void setup()
{
	// Initialize Serial communication and Slave Address
	Serial.begin(SERIAL_BAUD_RATE);
	deviceAddress = DEVICE_ADDRESS;
	serialMessageActive = 0;
	serialMessageLength = 0;
	serialMessageAvailable = 0;

	// Initialize Test Mode
	testHardwareSolenoidCount = 0;
	testHardwarePauseCount = 0;
}

// ****************************************************************************
// loop() - Main Program Loop Function 
// ****************************************************************************
void loop()
{
	if (HARDWARE_TEST)
	{
		hardwareTestLoop();
		return;
	}

	listenSerial();        // Listen for Serial Message
	processSerial();       // Process Serial Message and send to next Slave
	checkResetChannels();  // De-energize channels after time-out
}

// ****************************************************************************
// hardwareTestLoop() - In Hardware Test Mode, energize the solenoids in order 
// ****************************************************************************
void hardwareTestLoop()
{
	if (testHardwarePauseCount == 0)
	{
		// Energize the next solenoid
		setChannelsActive(1 << testHardwareSolenoidCount);
	}
	else
	{
		// De-energize all channels
		setChannelsActive(0);
	}

	// After the appropriate pause, move to the next solenoid
	testHardwarePauseCount++;
	if (testHardwarePauseCount == 10)
	{
		testHardwarePauseCount = 0;
		testHardwareSolenoidCount++;
		if (testHardwareSolenoidCount == 8)
		{
			testHardwareSolenoidCount = 0;
			delay(3000);
		}
	}
	delay(100);
}

// ****************************************************************************
// listenSerial() - Listen for the next Serial Message from the Master 
// ****************************************************************************
void listenSerial()
{
	while (Serial.available())
	{
		serialLastTime = millis();
		serialNextChar = Serial.read();
		if (serialMessageActive)
		{
			if (serialNextChar == SERIAL_MSG_END_CHAR)
			{
				// A complete serial message was detected
				listenSerialNewChar(0);
				serialMessageActive = 0;
				serialMessageAvailable = 1;
				return;
			}
			else
			{
				listenSerialNewChar(1);
			}
		}
		else 
		{
			if (serialNextChar == SERIAL_MSG_BEGIN_CHAR)
			{
				// The start of a new serial message was detected
				listenSerialNewChar(1);
				serialMessageActive = 1;
			}
			// Ignore any character that is not the Message Begin Character
		}
	}

	// If more than 1 second elapsed since the last serial data, then discard the
	// current buffer and start listening for new message
	if (millis() - serialLastTime > 1000)
	{
		serialMessageActive = 0;
		serialMessageLength = 0;
	}
}

// ****************************************************************************
// listenSerialNewChar() - A new character was detected while listening for
//   a serial message.  Store the new character in an array.
// leaveCharsAtEnd is used to require that the buffer leave 1 or more
//     characters at the end.
// ****************************************************************************
void listenSerialNewChar(uint8_t leaveCharsAtEnd)
{
	if (serialMessageLength >= SERIAL_MAX_DATA_LENGTH-leaveCharsAtEnd) return;
	serialMessageLength++;
	serialData[serialMessageLength-1] = serialNextChar;
}

// ****************************************************************************
// processSerial() - Used to process a Serial Message from the Master 
// ****************************************************************************
void processSerial()
{
	uint8_t addr;
	uint8_t cmd;
	if (!serialMessageAvailable) return;
	
	addr = readSerialByte(1);  // Slave Address in the Serial Message
	cmd = readSerialByte(3);   // Channel bitmask in the Serial Message
	
	// Energize/de-energize channels only if the address in the message
	// matches the current slave address
	if (addr == deviceAddress)
	{
		setChannelsActive(cmd);
	}

	// Transmit the same serial message so that the next Slave can see it
	Serial.write(serialData, serialMessageLength);
	Serial.println(F(""));

	// Begin listening for a new serial message
	serialMessageAvailable = 0;
	serialMessageLength = 0;
}

// ****************************************************************************
// readSerialByte() - Reads a byte from Hexadecimal text in the Serial Message
//                    starting at the position 'index'
// ****************************************************************************
uint8_t readSerialByte(uint8_t index)
{
	uint8_t n1;
	uint8_t n2;
	uint8_t b;

	n1 = charToNib(serialData[index]);
	n2 = charToNib(serialData[index+1]);
	b = n1 * 16 + n2;
	return b;
}

// ****************************************************************************
// charToNib() - Converts a single character from Hexidicimal text into
//               a Nibble (0 to 15)
// ****************************************************************************
uint8_t charToNib(uint8_t ch)
{
	if (ch >= '0' && ch <= '9') return (ch - '0');
	if (ch >= 'A' && ch <= 'F') return (ch - 'A' + 10);
	if (ch >= 'a' && ch <= 'f') return (ch - 'a' + 10);
	return 0;
}

// ****************************************************************************
// checkResetChannels() - If more than 1 second has elapsed since the last
//          command to energize or de-energize channels, then de-energize
//          all channels.  This is done so that the program will not be in a
//          state where solenoids are energized for a long time (possibly 
//          causing damage or overheating)
// ****************************************************************************
uint8_t checkResetChannels()
{
	if (millis() < setChannelsLastTime + 1000) return;
	setChannelsActive(0);  // De-energize all channels
}

// ****************************************************************************
// setChannelsActive() - Energizes channels per the 8-bit 'channelBitmask'
// ****************************************************************************
uint8_t setChannelsActive(uint8_t channelBitmask)
{
	setChannelsLastTime = millis();
	allChannelOutput(channelBitmask);
}

// ****************************************************************************
// allChannelOutput() - Sets the state of all channels per 'outputChannels'
//     which is an 8-bit bitmask of the channels to be energized.
// ****************************************************************************
void allChannelOutput(uint8_t outputChannels)
{
	uint8_t i;
	uint8_t motorChannels;

	// Convert the bitmask of 'outputChannels', which corresponds to the
	// shield terminal channels 0 through 7, to the bitmask of the
	// 8-bit bus after the 74HC595 shift register
	for (i = 0; i < 8; i++)
		bitWrite(motorChannels, motorBus[i], bitRead(outputChannels, i));

	// Send the 8-bit bitmask to the Shift register
	shiftWriteAll(motorChannels);

	// Set the PWM Pins.  Each PWM pin is set to 255 (fully-on) if either
	// the corresponding A or B is on, and set to 0 (fully off) otherwise
	for (i = 0; i < 4; i++)
	{
		if(bitRead(outputChannels,2*i) || bitRead(outputChannels, 2*i+1))
			analogWrite(motorPwmPins[i], 255);
		else
			analogWrite(motorPwmPins[i], 0);
	}
}

// ---------------------------------
// shiftWriteAll
//
// A variant of the shiftWrite function
// Pass it the bitMask of all channels to be written.
// There is no initialization function.
// Initialization is automatically done at the first
// time it is used.
//
void shiftWriteAll(int outputBitmask)
{
	static int latch_copy;
	static int shift_register_initialized = false;

	// Do the initialization on the fly, 
	// at the first time it is used.
	if (!shift_register_initialized)
	{
		// Set pins for shift register to output
		pinMode(MOTORLATCH, OUTPUT);
		pinMode(MOTORENABLE, OUTPUT);
		pinMode(MOTORDATA, OUTPUT);
		pinMode(MOTORCLK, OUTPUT);

		// Set pins for shift register to default value (low);
		digitalWrite(MOTORDATA, LOW);
		digitalWrite(MOTORLATCH, LOW);
		digitalWrite(MOTORCLK, LOW);
		// Enable the shift register, set Enable pin Low.
		digitalWrite(MOTORENABLE, LOW);

		// start with all outputs (of the shift register) low
		latch_copy = 0;

		shift_register_initialized = true;
	}

	// The defines HIGH and LOW are 1 and 0.
	// So this is valid.
	//bitWrite(latch_copy, output, high_low);
	latch_copy = outputBitmask;

	// Use the default Arduino 'shiftOut()' function to
	// shift the bits with the MOTORCLK as clock pulse.
	// The 74HC595 shiftregister wants the MSB first.
	// After that, generate a latch pulse with MOTORLATCH.
	shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);
	delayMicroseconds(5);    // For safety, not really needed.
	digitalWrite(MOTORLATCH, HIGH);
	delayMicroseconds(5);    // For safety, not really needed.
	digitalWrite(MOTORLATCH, LOW);
}



