//© 2023 Regents of the University of Minnesota. All rights reserved.

#include <HygroVue10.h>

HygroVue10::HygroVue10(SDI12Talon& talon_, uint8_t talonPort_, uint8_t sensorPort_, uint8_t version): talon(talon_)
{
	//Only update values if they are in range, otherwise stick with default values
	if(talonPort_ > 0) talonPort = talonPort_ - 1;
	else talonPort = 255; //Reset to null default if not in range
	if(sensorPort_ > 0) sensorPort = sensorPort_ - 1;
	else sensorPort = 255; //Reset to null default if not in range 
	sensorInterface = BusType::SDI12; 
}

String HygroVue10::begin(time_t time, bool &criticalFault, bool &fault)
{
	return ""; //DEBUG!
}

String HygroVue10::selfDiagnostic(uint8_t diagnosticLevel, time_t time)
{
	if(getSensorPort() == 0) throwError(FIND_FAIL); //If no port found, report failure
	else if(isPresent() == false) throwError(DETECT_FAIL); //If sensor port is good, but fail to detect sensor, throw error 
	String output = "\"HygroVue10\":{";
	if(diagnosticLevel == 0) {
		//TBD
	}

	if(diagnosticLevel <= 1) {
		//TBD
	}

	if(diagnosticLevel <= 2) {
		//TBD
	}

	if(diagnosticLevel <= 3) {
		//TBD
 	}

	if(diagnosticLevel <= 4) {
		//TBD
	}

	if(diagnosticLevel <= 5) {
		if(getSensorPort() != 0 && isPresent() == true) { //Test as normal
			String adr = talon.sendCommand("?!");
			int adrVal = adr.toInt();
			output = output + "\"Adr\":";
			if(adr.equals("") || (!adr.equals("0") && adrVal == 0)) output = output + "null"; //If no return, report null
			else output = output + adr; //Otherwise report the read value
			output = output + ",";
		}
		else output = output + "\"Adr\":null,"; //Else append null string
		
	}
	return output + "\"Pos\":[" + getTalonPortString() + "," + getSensorPortString() + "]}"; //Write position in logical form - Return compleated closed output
}

String HygroVue10::getMetadata()
{
	uint8_t adr = (talon.sendCommand("?!")).toInt(); //Get address of local device 
	String id = talon.command("I", adr);
	Serial.println(id); //DEBUG!
	String sdi12Version;
	String mfg;
	String model;
	String senseVersion;
	String sn;
	if((id.substring(0, 1)).toInt() != adr) { //If address returned is not the same as the address read, throw error
		Serial.println("ADDRESS MISMATCH!"); //DEBUG!
		//Throw error!
		sdi12Version = "null";
		mfg = "null";
		model = "null";
		senseVersion = "null";
		sn = "null";
	}
	else { //Standard across SDI-12 devices 
		sdi12Version = (id.substring(1,3)).trim(); //Grab SDI-12 version code
		mfg = (id.substring(3, 11)).trim(); //Grab manufacturer
		model = (id.substring(11,17)).trim(); //Grab sensor model name
		senseVersion = (id.substring(17,20)).trim(); //Grab version number
		sn = (id.substring(20,33)).trim(); //Grab the serial number 
	}
	String metadata = "\"HygroVue10\":{";
	metadata = metadata + "\"Hardware\":\"" + senseVersion + "\","; //Report sensor version pulled from SDI-12 system 
	metadata = metadata + "\"Firmware\":\"" + FIRMWARE_VERSION + "\","; //Static firmware version 
	metadata = metadata + "\"SDI12_Ver\":\"" + sdi12Version.substring(0,1) + "." + sdi12Version.substring(1,2) + "\",";
	metadata = metadata + "\"ADR\":" + String(adr) + ",";
	metadata = metadata + "\"Mfg\":\"" + mfg + "\",";
	metadata = metadata + "\"Model\":\"" + model + "\",";
	metadata = metadata + "\"SN\":\"" + sn + "\",";
	metadata = metadata + "\"Pos\":[" + getTalonPortString() + "," + getSensorPortString() + "]"; //Concatonate position 
	metadata = metadata + "}"; //CLOSE  
	return metadata; 
}

String HygroVue10::getData(time_t time)
{
	String output = "\"HygroVue10\":{"; //OPEN JSON BLOB
	bool readDone = false;
	delay(1000); //DEBUG!
	if(getSensorPort() != 0) { //Check both for detection 
		for(int i = 0; i < talon.retryCount; i++) {
			if(!isPresent()) continue; //If presence check fails, try again

			int adr = talon.getAddress();
			if(adr < 0) {
				continue; //If address is out of range, try again
			}
			int waitTime = talon.startMeasurmentCRC(adr, 3); //Send MC3 command to read all values in 
			if(waitTime <= 0) {
				continue; //If wait time out of range, try again
			}

			delay(waitTime*1000 + 500); //Wait for number of seconds requested, plus half a second to make sure
			String data = talon.command("D0", adr); //Have sensor return data 

			if(!talon.testCRC(data)) continue; //If CRC is bad, try again

			float sensorData[4] = {0.0}; //Store the 4 vals from the sensor in float form
			if((data.substring(0, indexOfSep(data))).toInt() != adr) { //If address returned is not the same as the address read, throw error
				throwError(talon.SDI12_SENSOR_MISMATCH | 0x100 | talonPortErrorCode | sensorPortErrorCode); //Throw error on address change, this is a weird place for this error to happen, but could
				continue; //Try again
			}
			parseData(data, sensorData, 4); //Parse each value from string into data array

			output = output + appendData(sensorData[0], "Temperature", 3); //Append temp with 3 decimal places (see datasheet for reported resolution)
			output = output + appendData(sensorData[1], "Humidity", 3); //Append humidity with 3 decimal places (see datasheet for reported resolution)
			output = output + appendData(sensorData[2], "Dewpoint", 3); //Append dew point with 3 decimal places (see datasheet for reported resolution)
			output = output + appendData(sensorData[3], "VaporPressure", 3, false); //Append vapor pressure with 3 decimal places (see datasheet for reported resolution), ignore trailing comma for last entry

			if(sensorData[0] == -99.999 && sensorData[1] == -99.999) throwError(HYGROVUE_SENSE_FAIL | talonPortErrorCode | sensorPortErrorCode); //Fault with sensor element indicated, see datasheet for details

			readDone = true; //Set flag
			break; //Stop retry
		}	
		if(readDone == false) throwError(talon.SDI12_READ_FAIL); //Only throw read fail error if sensor SHOULD be detected 
	}
	else throwError(FIND_FAIL);
	if(getSensorPort() == 0 || readDone == false) output = output + "\"Temperature\":null,\"Humidity\":null,\"Dewpoint\":null,\"VaporPressure\":null"; //Append nulls if no sensor port found, or read did not work
	output = output + ",\"Pos\":[" + getTalonPortString() + "," + getSensorPortString() + "]"; //Concatonate position 
	output = output + "}"; //CLOSE JSON BLOB
	Serial.println(output); //DEBUG!
	return output;
}

bool HygroVue10::isPresent() 
{ 
	uint8_t adr = (talon.sendCommand("?!")).toInt();
	
	String id = talon.command("I", adr);
	id.remove(0, 1); //Trim address character from start
	Serial.print("SDI12 Address: "); //DEBUG!
	Serial.print(adr);
	Serial.print(",");
	Serial.println(id);
	if(id.indexOf("HVUE") > 0) return true; //FIX! Check version here!
	else return false;
}

int HygroVue10::indexOfSep(String input) //Find next location of +/- deliminater character
{
	int pos1 = input.indexOf('+');
	int pos2 = input.indexOf('-');
	if(pos1 >= 0 && pos2 >= 0) return min(pos1, pos2); //If both are positive, just return the straight min
	else return max(pos1, pos2); //If one of them is -1, then return the other one. If both are -1, then you should return -1 anyway
}

String HygroVue10::appendData(float data, String label, uint8_t precision, bool appendComma)
{
	String val = "";
	if(data == -9999 || data == 9999999) val = "\"" + label + "\":null"; //Append null if value is error indicator
	else val = "\"" + label + "\":" + String(data, precision); //Otherwise, append as normal using fixed specified precision

	if(appendComma) return val + ",";
	else return val;
}

bool HygroVue10::parseData(String input, float dataReturn[], uint8_t dataLen)
{
	const uint8_t strLen = input.length(); //Get length of string to make char array
	char inputArr[strLen] = {0}; //Initialize to zero
	input.toCharArray(inputArr, strLen); //Copy to array

	uint8_t numSeps = 0; //Keep track of number of seperators found
	for(int i = 0; i < strLen; i++){
		if(inputArr[i] == '+' or inputArr[i] == '-') numSeps += 1; //Increment seperator count if either +/- is found (Note: CRC vals to not contain + or -)
	}
	if(numSeps != dataLen) return false; //Return error if number of seperators does not match the requested number of values
	
	input.remove(0, 1); //Delete address from start of string
	for(int i = 0; i < dataLen; i++) { //Parse string into floats -- do this to run tests on the numbers themselves and make sure formatting is clean
		if(indexOfSep(input) == 0 && indexOfSep(input.substring(indexOfSep(input) + 1)) > 0) { //If string starts with seperator AND this is not the last seperator 
			dataReturn[i] = (input.substring(0, indexOfSep(input.substring(indexOfSep(input) + 1)) + 1)).toFloat(); //Extract float from between next two seperators
			// Serial.println(input.substring(0, indexOfSep(input.substring(indexOfSep(input) + 1)) + 1)); //DEBUG!
			input.remove(0, indexOfSep(input.substring(indexOfSep(input) + 1)) + 1); //Delete leading entry
		}
		else {
			input.trim(); //Trim off trailing characters
			dataReturn[i] = input.toFloat();
		}
	}
	return true; //Give pass
}


String HygroVue10::getErrors()
{
	String output = "\"HygroVue10\":{"; // OPEN JSON BLOB
	output = output + "\"CODES\":["; //Open codes pair

	for(int i = 0; i < min(MAX_NUM_ERRORS, numErrors); i++) { //Interate over used element of array without exceeding bounds
		output = output + "\"0x" + String(errors[i], HEX) + "\","; //Add each error code
		errors[i] = 0; //Clear errors as they are read
	}
	if(output.substring(output.length() - 1).equals(",")) {
		output = output.substring(0, output.length() - 1); //Trim trailing ','
	}
	output = output + "],"; //close codes pair
	output =  output + "\"OW\":"; //Open state pair
	if(numErrors > MAX_NUM_ERRORS) output = output + "1,"; //If overwritten, indicate the overwrite is true
	else output = output + "0,"; //Otherwise set it as clear
	output = output + "\"NUM\":" + String(numErrors) + ","; //Append number of errors
	output = output + "\"Pos\":[" + getTalonPortString() + "," + getSensorPortString() + "]"; //Concatonate position 
	output = output + "}"; //CLOSE JSON BLOB
	numErrors = 0; //Clear error count
	return output;
}