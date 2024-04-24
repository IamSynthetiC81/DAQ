/**
 * @file ErrorHandler.h
 * 
 * @brief This file contains the error handling functions and error codes for the project.
 *  
*/


#include <stdint.h>
#include <avr/pgmspace.h>


const PROGMEM uint8_t NESTED_VECTORED_ERROR_CONTROLLER[8];

#define REGISTER_GPS_ERROR 0
#define REGISTER_IMU_ERROR 1
#define REGISTER_SDC_ERROR 2
#define REGISTER_ARD_ERROR 3
#define REGISTER_INT_ERROR 4
#define REGISTER_TIM_ERROR 5
#define REGISTER_GN1_ERROR 6
#define REGISTER_GN2_ERROR 7

typedef enum GPS_ERROR_CODES{
    BAUD_RATE_MISMATCH = 0,
    PORT_INACTIVE = 1,
    DATA_CORRUPT = 2,
    DATA_MISSING = 4,
    DATA_OUT_OF_BOUNDS = 5,
} GPS_ERROR_CODES;

typedef enum IMU_ERROR_CODES{
    ADDRESS_NOT_FOUND = 0,
    DATA_CORRUPT = 1,
    DATA_MISSING = 2,
    DATA_OUT_OF_BOUNDS = 3,
} IMU_ERROR_CODES;


void 