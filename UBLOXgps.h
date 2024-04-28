// #include <SparkFun_u-blox_GNSS_Arduino_Library.h>
// #include <u-blox_config_keys.h>
// #include <u-blox_structs.h>

// #include "HardwareSerial.h"



// void sendPacket(byte *packet, byte len){
//   for (byte i = 0; i < len; i++){
//     Serial1.write(pgm_read_byte(packet[i]));
//     delay(5);
//   }
// }

// void sendCommand(byte *command, byte len){
//   sendPacket(command, len);
// }

// void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct)
// {
    
//     __LOG.Latitude = ubxDataStruct->lat;
//     __LOG.Longitude = ubxDataStruct->lon
//     __LOG.Altitude = ubxDataStruct->hMSL;
//     __LOG.GroundSpeed = myGNSS.getGroundSpeed();
//     __LOG.Heading = myGNSS.getHeading();
    
//     // Serial.println();

//     // Serial.print(F("Time: ")); // Print the time
//     // uint8_t hms = ubxDataStruct->hour; // Print the hours
//     // if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
//     // Serial.print(hms);
//     // Serial.print(F(":"));
//     // hms = ubxDataStruct->min; // Print the minutes
//     // if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
//     // Serial.print(hms);
//     // Serial.print(F(":"));
//     // hms = ubxDataStruct->sec; // Print the seconds
//     // if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
//     // Serial.print(hms);
//     // Serial.print(F("."));
//     // unsigned long millisecs = ubxDataStruct->iTOW % 1000; // Print the milliseconds
//     // if (millisecs < 100) Serial.print(F("0")); // Print the trailing zeros correctly
//     // if (millisecs < 10) Serial.print(F("0"));
//     // Serial.print(millisecs);

//     // long latitude = ubxDataStruct->lat; // Print the latitude
//     // Serial.print(F(" Lat: "));
//     // Serial.print(latitude);

//     // long longitude = ubxDataStruct->lon; // Print the longitude
//     // Serial.print(F(" Long: "));
//     // Serial.print(longitude);
//     // Serial.print(F(" (degrees * 10^-7)"));

//     // long altitude = ubxDataStruct->hMSL; // Print the height above mean sea level
//     // Serial.print(F(" Height above MSL: "));
//     // Serial.print(altitude);
//     // Serial.println(F(" (mm)"));
// }

// bool initGPS(SFE_UBLOX_GNSS *myGNSS) {
//   Serial1.begin(9600);

//   Stream *gps; 
//   gps = &Serial1;

//   if (myGNSS->begin(*gps, 250, false) == false){
//     Serial.println(F("GPS not detected\n"));
//     return false;
//   }


//   // send configuration data in UBX protocol
//   for(int i = 0; i < sizeof(CONFIG); i++) {                        
//     Serial1.write( pgm_read_byte(CONFIG+i) );
//     delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
//   }

//   Serial1.setTimeout(100);

//   // Serial1.flush();
//   // Serial1.begin(115200);

//   myGNSS->setUART1Output(COM_TYPE_UBX);
//   myGNSS->saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
//   myGNSS->setAutoPVTcallbackPtr(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata
// }