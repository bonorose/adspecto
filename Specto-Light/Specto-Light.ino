/*
 Name:		Specto_Light.ino
 Created:	14-Sep-19 11:12:20 AM
 Author:	Simeon Marlokov
 Brief: This project contains the source code for the Specto Light prototype testing module.
 Hardware Components: 
	Solar Panel; 
	BME280 Temperature Sensor, 
	5V Relay JQC-3FF-S-Z,
	TSL 2591 Dynamic Light Sensor
 Microcontroller: Arduino Nano 328p + NodeMCU ESP 8266
*/

// Library Initialization
#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_TSL2591_Library/Adafruit_TSL2591.h>

// Parameter & Variable Declaration/Initialization

// A3 and A4 for SCL + SDA

LiquidCrystal_I2C lcd(0x27, 20, 4);  // Initialize a LCD with I2C address set to 0x27 for a 16 chars and 2 line display
Adafruit_BME280 BME; // Initializing BME sensor as I2C device
Adafruit_TSL2591 TSL = Adafruit_TSL2591(2591);  // Initializing TSL 2591 dynamic light sensor as I2C device.

int Relay = 3;
int Solar = A2;


/** @brief Read analog solar panel callback funciton.
 *  @param Pin int
 *  @return float Read Solar voltage.
 */
float SolarMeasure() {

}

/** @brief Initialize the LCD and write a test message.
 *  @return Initialized LCD ready for writing.
 */
void DisplayInit() {
	// Initialize the lcd. 
	lcd.init();
	lcd.init();

	// Test Print LCD.
	lcd.backlight();
	lcd.setCursor(0, 0);
	lcd.print("AdSpecto Inc.");
	
}

/** @brief Control the relay and turn on/off the lightbulb based on time.
 *  @param Time from RTC.
 *  @return bool Bulb Status.
 */
void RelaySwitch() {
	
}

/** @brief Set TSL 2591 working parameters.
 *  @param void
 *  @return Configured Lux Sensor :).
 */
void LuxSetup(void)
{
	// You can change the gain on the fly, to adapt to brighter/dimmer light situations
	TSL.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
	//TSL.setGain(TSL2591_GAIN_MED);      // 25x gain

	// Changing the integration time gives you a longer time over which to sense light
	// longer timelines are slower, but are good in very low light situtations!
	TSL.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
	//TSL.setTiming(TSL2591_INTEGRATIONTIME_200MS);
}


/** @brief Read analog solar panel callback funciton.
 *  @param .
 *  @return uint16_t Calculated lux.
 */
int LuxRead(void)
{
	// More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum.
	// That way you can do whatever math and comparisons you want!
	uint32_t lum = TSL.getFullLuminosity();
	uint16_t ir, full;
	ir = lum >> 16;
	full = lum & 0xFFFF;

	uint16_t lux = TSL.calculateLux(full, ir); //Calculates total lux summing both IR and FS.
	return lux;
}


/*
TO-DO: Can seperate parameters and variables in a seperate file
*/

/** @brief Initializes all of the nessecary pins for the project
 */
void PinMeBaby() {
	pinMode(Relay, OUTPUT); // sets the relay pin to output
}

/** @brief Check if all the sensors have initialized correctly. If not, flag which sensor and stop program.
 *  @param Actual hardware.
 *  @return bool check if working.
 */
void StatusCheck() {
	bool status;

	Serial.print("Checking BME sensor.");
	status = BME.begin();
	if (!status) {
		Serial.println("Could not find a valid BME280 sensor, check wiring!");
		while (1);
	}

	Serial.print("Checking Lux sensor.");
	status = lux.begin();
	if (!status) {
		Serial.println("Could not find a valid TSL 2591 sensor, check wiring!");
		while (1);
	}
		
}



// the setup function runs once when you press reset or power the board
void setup() {
	//Serial.begin(9600); //Initializes USB serial communication with host computer at a baud of 9600

	DisplayInit();
	LuxSetup();
	
	PinMeBaby();

	//StatusCheck();
}

// the loop function runs over and over again until power down or reset
void loop() {
	SolarMeasure();
	BME.readTemperature();
}