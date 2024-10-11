//© 2023 Regents of the University of Minnesota. All rights reserved.

#ifndef HYGROVUE10_h
#define HYGROVUE10_h

#include <Sensor.h>
#include <SDI12Talon.h>

class HygroVue10: public Sensor
{
	constexpr static int DEAFULT_PORT = 2; ///<Use port 2 by default
	constexpr static int DEFAULT_SENSOR_PORT = 0; ///<Use port 0 by default
  	constexpr static int DEFAULT_VERSION = 0x00; ///<Use hardware version unknown by default
	const String FIRMWARE_VERSION = "0.0.0"; //FIX! Read from system??

	const uint32_t HYGROVUE_SENSE_FAIL = 0xA0030000; //Sensor failure on board the HygroVue10 - sensor or internal communication failure

	public:
		HygroVue10(SDI12Talon& talon_, uint8_t talonPort_ = DEAFULT_PORT, uint8_t sensorPort_ = DEFAULT_SENSOR_PORT, uint8_t version = DEFAULT_VERSION);
		String begin(time_t time, bool &criticalFault, bool &fault);
		String getData(time_t time);
		String selfDiagnostic(uint8_t diagnosticLevel = 4, time_t time = 0); //Default to just level 4 diagnostic, default to time = 0
		String getMetadata();
		String getErrors();
		bool isPresent();

	private:
		SDI12Talon& talon;
		String appendData(float data, String label, uint8_t precision = 2, bool appendComma = true); //Default to precision of 2
		bool parseData(String input, float dataReturn[], uint8_t dataLen);
		int indexOfSep(String input);

		bool initDone = false; //Used to keep track if the initaliztion has run - used by hasReset() 

		uint8_t version = 0; //FIX! This should be read from EEPROM in future 
};

#endif