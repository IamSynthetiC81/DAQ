#ifndef SD_H
#define SD_H

#include <SD.h>
#include <string.h>
#include <stdint.h>

#define BUFFER_MAX_SIZE 1024

class MySD {
	public:
    MySD(uint8_t csPin);
    bool init(const char* name);
		bool log(const char* packet, size_t len);
		bool close();
		
	private:
		char FileName[40];

		uint8_t csPin;
		
		char buffer[BUFFER_MAX_SIZE];
		size_t buffer_len = 0;

		

		bool flushBuffer();
};

#endif // SD_H
