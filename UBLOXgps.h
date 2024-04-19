// #include "Stream.h"
// #include "HardwareSerial.h"
// #include "ErrorHandler.h"


// HardwareSerial *GPS;
// HardwareSerial *LOG;

// const PROGMEM uint32_t baudRates[] = {9200, 19200, 38400, 57600, 115200};

// void GPS_init(HardwareSerial *GPS_port, Stream *LOG_port, uint32_t baud_rate){

// }

// uint32_t autoBaudRate(){
//   long baudRates[] = {9600, 19200, 38400, 57600, 115200};
//   int numBaudRates = sizeof(baudRates) / sizeof(long);

//   for (int i = 0; i < numBaudRates; i++) {
//     GPS->flush();
//     GPS->begin(baudRates[i]);
//     delay(1000); // Wait for data to arrive


//     if (LOG->available()) {
//       return baudRates[i];
//     }
//   }

//   NESTED_VECTORED_ERROR_REGISTERS[REGISTER_GPS_ERROR] |= (1 <<ERROR_BAUD_RATE_MISMATCH );
//   return 0xffff;
// }