#include "SD.h"

#include <string.h>
#include <stdint.h>

#define BUFFER_MAX_SIZE 1024

MySD::MySD(uint8_t csPin) {
	this->csPin = csPin;
}

bool MySD::init(const char* name) {
	memset(this->buffer, '\0', BUFFER_MAX_SIZE);
	this->buffer_len = 0;

	SD.begin(this->csPin);

	if (!SD.begin(this->csPin)) {
		return false;
	}

	strncpy(this->FileName, name, sizeof(this->FileName) - 1);  // Ensure null termination
	this->FileName[sizeof(this->FileName) - 1] = '\0';

	char FileNum[] = "00";
	strncat(FileName, FileNum, sizeof(FileName) - strlen(FileName) - 1);

	// append .csv to the file name
	strncat(this->FileName, ".txt", sizeof(this->FileName) - strlen(this->FileName) - 1);

	// Check if the file exists
	while (SD.exists(FileName)) {
		if (FileNum[1] == '9') {
			if (FileNum[0] == '9') return false;
			
			FileNum[0]++;
			FileNum[1] = '0';
		} else {
			FileNum[1]++;
		}

		strncpy(this->FileName, name, sizeof(this->FileName) - 1);  // Reset FileName
		this->FileName[sizeof(this->FileName) - 1] = '\0';
		strncat(this->FileName, FileNum, sizeof(this->FileName) - strlen(this->FileName) - 1);
		// append .csv to the file name
		strncat(this->FileName, ".txt", sizeof(this->FileName) - strlen(this->FileName) - 1);
	}

	File myFile = SD.open(this->FileName, FILE_WRITE);
	if (!myFile) {
		Serial.println("Failed to open file");
		return false;
	} else {
		myFile.println("accelerationX,accelerationY,accelerationZ,gyroX,gyroY,gyroZ,Vref,BrakePressure,ThrottlePositionSensor,SteeringWheel,CounterPulses,GroundSpeed,Heading,Latitude,Longitude,Altitude,elapsedTime");
		myFile.close();

		Serial.print("File name : ");
		Serial.println(FileName);
	}

	return true;
}
bool MySD::log(const char* packet, size_t len) {
	if (this->buffer_len + len >= BUFFER_MAX_SIZE) {
		if (!this->flushBuffer()) {
			return false;
		}
	}

	memcpy(this->buffer + this->buffer_len, packet, len);
	this->buffer_len += len;

	return true;
}
bool MySD::flushBuffer() {
	File myFile = SD.open(this->FileName, FILE_WRITE);
	if (!myFile) {
		Serial.println("Failed to open file");
		return false;
	}

	unsigned int bytesWriten = myFile.write(this->buffer, this->buffer_len);
	if (bytesWriten != this->buffer_len) {
		Serial.println("Failed to write to file");
		myFile.close();
		return false;
	}

	memset(this->buffer, '\0', BUFFER_MAX_SIZE);
	this->buffer_len = 0;

	myFile.close();

	return true;
}
bool MySD::close() {
	if (this->buffer_len > 0) {
		if (!this->flushBuffer()) {
			return false;
		}
	}

	return true;
}