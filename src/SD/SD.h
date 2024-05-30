#ifndef SD_H
#define SD_H

#include <FreeStack.h>
#include <SdFat.h>

#include "../Definitions/Packet.h"

//------------------------------------------------------------------------------
// Set ENABLE_ERROR_HANDLING nonzero to enable error handling through the SD_error.h library.
#define ENABLE_ERROR_HANDLING 1
#if ENABLE_ERROR_HANDLING
	#include "SD_error.h"
	#include "../ErrorRegister/ErrorRegister.h"
	extern ErrorRegister ERROR_REGISTER_SD;
#endif  // ENABLE_ERROR_HANDLING


//------------------------------------------------------------------------------
// This example was designed for exFAT but will support FAT16/FAT32.
//
// Note: Uno will not support SD_FAT_TYPE = 3.
// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 2
//------------------------------------------------------------------------------
// Set USE_RTC nonzero for file timestamps.
// RAM use will be marginal on Uno with RTClib.
// Set USE_RTC nonzero for file timestamps.
// RAM use will be marginal on Uno with RTClib.
// 0 - RTC not used
// 1 - DS1307
// 2 - DS3231
// 3 - PCF8523
#define USE_RTC 0
#if USE_RTC
#include "RTClib.h"
#endif  // USE_RTC
//------------------------------------------------------------------------------
// Pin definitions.
//
// Digital pin to indicate an error, set to -1 if not used.
// The led blinks for fatal errors. The led goes on solid for SD write
// overrun errors and logging continues.
const int8_t ERROR_LED_PIN = -1;

// SD chip select pin.
const uint8_t SD_CS_PIN = SS;
//------------------------------------------------------------------------------
// File definitions.
//
// Maximum file size in bytes.
// The program creates a contiguous file with MAX_FILE_SIZE_MiB bytes.
// The file will be truncated if logging is stopped early.
const uint32_t MAX_FILE_SIZE_MiB = 100;  // 100 MiB file.

// Maximum length name including zero byte.
const size_t NAME_DIM = 40;
//------------------------------------------------------------------------------
// FIFO size definition. Use a multiple of 512 bytes for best performance.
//
#if RAMEND < 0X8FF
#error SRAM too small
#elif RAMEND < 0X10FF
const size_t FIFO_SIZE_BYTES = 512;
#elif RAMEND < 0X20FF
const size_t FIFO_SIZE_BYTES = 4 * 512;
#elif RAMEND < 0X40FF
const size_t FIFO_SIZE_BYTES = 12 * 512;
#else   // RAMEND
const size_t FIFO_SIZE_BYTES = 16 * 512;
#endif  // RAMEND
//==============================================================================
const uint32_t MAX_FILE_SIZE = MAX_FILE_SIZE_MiB << 20;

// Max SPI rate for AVR is 10 MHz for F_CPU 20 MHz, 8 MHz for F_CPU 16 MHz.
#define SPI_CLOCK SD_SCK_MHZ(8)

// Select fastest interface.
#if ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else  // ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif  // ENABLE_DEDICATED_SPI`
//==============================================================================
// Preallocate 500MiB file.
const uint32_t PREALLOCATE_SIZE_MiB = 512UL;

// FIFO SIZE - 512 byte sectors.  Modify for your board.
#ifdef __AVR_ATmega328P__
// Use 512 bytes for 328 boards.
#define FIFO_SIZE_SECTORS 1
#elif defined(__AVR__)
// Use 2 KiB for other AVR boards.
#define FIFO_SIZE_SECTORS 4
#else  // __AVR_ATmega328P__
// Use 8 KiB for non-AVR boards.
#define FIFO_SIZE_SECTORS 16
#endif  // __AVR_ATmega328P__
//==============================================================================

const uint64_t PREALLOCATE_SIZE = (uint64_t)PREALLOCATE_SIZE_MiB << 20;
// Max length of file name including zero byte.
#define FILE_NAME_DIM 40
// Max number of records to buffer while SD is busy.
const size_t FIFO_DIM = 512 * FIFO_SIZE_SECTORS / sizeof(Packet);

#if SD_FAT_TYPE == 0
typedef SdFat sd_t;
typedef File file_t;
#elif SD_FAT_TYPE == 1
typedef SdFat32 sd_t;
typedef File32 file_t;
#elif SD_FAT_TYPE == 2
typedef SdExFat sd_t;
typedef ExFile file_t;
#elif SD_FAT_TYPE == 3
typedef SdFs sd_t;
typedef FsFile file_t;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

class SD{
	public:
    SD(const char* name);
		~SD();
    
		bool logData(Packet data);
		
	private:
		file_t csvFile;

		Packet fifo[FIFO_DIM];
		uint16_t fifoHead = 0;

		char LOG_FILE_NAME[13];
		SdFat sd;

		void Write(Packet *data, unsigned int size);

		bool createCsvFile(const char* name);
};

#endif  // SD_H