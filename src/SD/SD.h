#ifndef SD_H
#define SD_H

#include <SD.h>
#include <string.h>
#include <stdint.h>

#define BUFFER_MAX_SIZE 1024

class MySD {
public:
    MySD(uint8_t csPin) {
        this->csPin = csPin;
    }

    bool init(const char* name) {
			memset(this->buffer, '\0', BUFFER_MAX_SIZE);
			this->buffer_len = 0;

			int ff = SD.begin(this->csPin);

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

			this->myFile = SD.open(this->FileName, FILE_WRITE);
			if (!myFile) {
				Serial.println("Failed to open file");
				return false;
			} else {
				this->myFile.println("accelerationX,accelerationY,accelerationZ,gyroX,gyroY,gyroZ,Vref,BrakePressure,ThrottlePositionSensor,SteeringWheel,CounterPulses,GroundSpeed,Heading,Latitude,Longitude,Altitude,elapsedTime");
				this->myFile.close();

				Serial.print("File name : ");
				Serial.println(FileName);
			}
			return true;
    }

		/**
		 * @brief Open the file, write the packet to the buffer,flush the buffer to the file when full, and close the file
		*/
    bool log(const char* packet, size_t len) {
			if(this->buffer_len + len > BUFFER_MAX_SIZE){
				if (!this->myFile){
					Serial.println("File not opened");
					Serial.print("Opening file ");
					Serial.println(this->FileName);
					this->myFile = SD.open(this->FileName, FILE_WRITE);
					if (!this->myFile) {
						Serial.println("Failed to open file");
						return false;
					}

					Serial.println("File opened");

					this->myFile.write("Hello\n", len);
					// this->myFile.write(packet, len);

					this->myFile.close();

					Serial.println("File opened");

					return true;
				} else {
					this->myFile.write(this->buffer, this->buffer_len);
					this->buffer_len = 0;

					this->myFile.write(packet, len);

					this->myFile.close();

					Serial.println("File opened");

					return true;

				}
			}else {
				memcpy(this->buffer + this->buffer_len, packet, len);
				this->buffer_len += len;

				Serial.println("Buffered");

				return true;
			}
    }

		bool write(const char* packet, size_t len) {
			unsigned int chunkSize = myFile.availableForWrite();
			if (chunkSize && len >= chunkSize) {
				myFile.write(packet, chunkSize);
				memset(this->buffer, '\0', BUFFER_MAX_SIZE);
			}
		}

    bool flushBuffer() {
			if (!this->myFile) {
				this->myFile = SD.open(this->FileName, FILE_WRITE);
				if (!this->myFile) {
					Serial.println("Failed to open file");
					return false;
				}

				Serial.println("File opened");

				if (this->buffer_len > 0) {
					this->myFile.write(this->buffer, this->buffer_len);
					this->buffer_len = 0;
				}

				this->myFile.close();

				return true;
			} else {
				if (this->buffer_len > 0) {
					this->myFile.write(this->buffer, this->buffer_len);
					this->buffer_len = 0;
				}

				return true;				
    	}
		}

    void close() {
        flushBuffer();
        if (this->myFile) {
            this->myFile.close();
        }
    }

private:
    File myFile;
    uint8_t csPin;
    char buffer[BUFFER_MAX_SIZE];
    size_t buffer_len = 0;

		char FileName[40];
};

#endif // SD_H
