#pragma once

#include <stdint.h>
#include <string.h>

class Packet{
	public:
		Packet(){
			accelerationX = 0;
			accelerationY = 0;
			accelerationZ = 0;
			gyroX = 0;
			gyroY = 0;
			gyroZ = 0;
			Vref = 0;
			BrakePressure = 0;
			ThrottlePositionSensor = 0;
			SteeringWheel = 0;
			CounterPulses = 0;
			GroundSpeed = 0;
			Heading = 0;
			Latitude = 0;
			Longitude = 0;
			Altitude = 0;
			elapsedTime = 0;
		}
		
		int16_t accelerationX;
		int16_t accelerationY;
		int16_t accelerationZ;
		int16_t gyroX;
		int16_t gyroY;
		int16_t gyroZ;
		uint16_t Vref;
		uint16_t BrakePressure;
		uint16_t ThrottlePositionSensor;
		uint16_t SteeringWheel;
		uint16_t CounterPulses;
		long GroundSpeed;
		long Heading;
		long Latitude;
		long Longitude;
		long Altitude;
		uint64_t elapsedTime;

		uint16_t toChar(char* buffer){
			uint16_t i = 0;
			i += sprintf(buffer + i, "%d,", accelerationX);
			i += sprintf(buffer + i, "%d,", accelerationY);
			i += sprintf(buffer + i, "%d,", accelerationZ);
			i += sprintf(buffer + i, "%d,", gyroX);
			i += sprintf(buffer + i, "%d,", gyroY);
			i += sprintf(buffer + i, "%d,", gyroZ);
			i += sprintf(buffer + i, "%d,", Vref);
			i += sprintf(buffer + i, "%d,", BrakePressure);
			i += sprintf(buffer + i, "%d,", ThrottlePositionSensor);
			i += sprintf(buffer + i, "%d,", SteeringWheel);
			i += sprintf(buffer + i, "%d,", CounterPulses);
			i += sprintf(buffer + i, "%ld,", GroundSpeed);
			i += sprintf(buffer + i, "%ld,", Heading);
			i += sprintf(buffer + i, "%ld,", Latitude);
			i += sprintf(buffer + i, "%ld,", Longitude);
			i += sprintf(buffer + i, "%ld,", Altitude);
			i += sprintf(buffer + i, "%ld,", elapsedTime);
			return i;
		}

		uint8_t MAX_STR_SIZE(){
			uint8_t num_of_int16_t = 6;
			uint8_t num_of_unsigned_int16_t = 5;
			uint8_t num_of_uint64_t = 1;
			uint8_t num_of_long = 5;
			
			size_t size = 0;

			char buffer[15];
			snprintf(buffer, 15, "%d,", INT16_MAX);
			size += (strlen(buffer)*num_of_int16_t);

			snprintf(buffer, 15, "%d,", UINT16_MAX);
			size += (strlen(buffer)*num_of_unsigned_int16_t);

			snprintf(buffer, 15, "%ld,", UINT64_MAX);
			size += (strlen(buffer)*num_of_uint64_t);

			snprintf(buffer, 15, "%ld,", INT64_MAX);
			size += (strlen(buffer)*num_of_long);

			return size;
		}
};