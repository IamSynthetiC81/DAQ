// class Printer{
//   public: 
//     void *SerialPrint(message_t);
//     int paparigas = 0;
//   private :
//     void SerialPrintGSA(GSA_t message){
//       Serial.print(F("Header : "));
//       Serial.println(message.Header);
//       Serial.print(F("Mode 1 : "));
//       Serial.println(message.Mode1);
//       Serial.print(F("Mode 2 : "));
//       Serial.println(message.Mode2);
//       Serial.print(F("HDOP : "));
//       Serial.println(message.HDOP);
//       Serial.print(F("PDOP : "));
//       Serial.println(message.PDOP);
//       Serial.print(F("VDOP : "));
//       Serial.println(message.VDOP);
//       for (int i = 0 ; i < 12 ; i++ ){
//         Serial.print(F("PRN["));
//         Serial.print(i);
//         Serial.print(F("] : "));
//         Serial.println(message.PRN[i]);
//       }
//       Serial.print(F("CHECKSUM : "));
//       Serial.println(message.checksum);
//     }
//     void SerialPrintGLL(GLL_t message){
//       Serial.print(F("Header : "));
//       Serial.println(message.Header);
//       Serial.print(F("Latitude : "));
//       Serial.println(message.Latitude);
//       Serial.print(F("N/S Indicator : "));
//       Serial.println(message.NSIndicator);
//       Serial.print(F("Longitude : "));
//       Serial.println(message.Longitude);
//       Serial.print(F("E/W Indicator : "));
//       Serial.println(message.EWIndicator);
//       Serial.print(F("UTC Time : "));
//       Serial.println(message.UTCtime);
//       Serial.print(F("Status : "));
//       Serial.println(message.Status);
//       Serial.print(F("Mode : "));
//       Serial.println(message.Mode);
//       Serial.print(F("Checksum : "));
//       Serial.println(message.checksum);
//     }
//     void SerialPrintRMC(RMC_t message){
//       Serial.print(F("Header : "));
//       Serial.println(message.Header);
//       Serial.print(F("UTC Time : "));
//       Serial.println(message.UTCtime);
//       Serial.print(F("Status : "));
//       Serial.println(message.Status);
//       Serial.print(F("Latitude : "));
//       Serial.println(message.Latitude);
//       Serial.print(F("N/S Indicator  : "));
//       Serial.println(message.NSIndicator);
//       Serial.print(F("Longitude : "));
//       Serial.println(message.Longitude);
//       Serial.print(F("E/W Indicator  : "));
//       Serial.println(message.EWIndicator);
//       Serial.print(F("Speed Over Ground : "));
//       Serial.println(message.SpeedOverGround);
//       Serial.print(F("Course Over Ground : "));
//       Serial.println(message.CourseOverGround);
//       Serial.print(F("Date : "));
//       Serial.println(message.Date);
//       Serial.print(F("Magnetic Variation : "));
//       Serial.println(message.MagneticVariation);
//       Serial.print(F("East/West Indicator : "));
//       Serial.println(message.VariationSense);
//       Serial.print(F("Mode : "));
//       Serial.println(message.Mode);
//       Serial.print(F("Checksum : "));
//       Serial.println(message.checksum);
//     }

// }