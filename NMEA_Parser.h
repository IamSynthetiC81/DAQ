#include <string.h>
#include <stdlib.h>

typedef enum format{
  GSA,
  GSV,
  GLL,
  RMC,
  GGA,
  MSS,
  ZDA,
  VTG
} frmt_t;

typedef struct GSA{
  char Header[10];
  char Mode1;               /*  M for Manual : Forced to operate in 2D or 3D mode
                                A for 2D Automatic : Allowd to automatically switch 2D/3D
                            */
  uint8_t Mode2;            /*  1 : Fix not available
                                2 : 2D (<4SV's Used)
                                3 : 3D (>3 SV's Used)
                            */
  uint8_t PRN[12];          // Satellites Used
  float PDOP;               // Position Dilution of Precision
  float HDOP;               // Horizontal Dilution of Precision
  float VDOP;               // Vertical Dilution of Precision
  char checksum[6];
} GSA_t;
typedef struct GSV{
  char Header[10];
  uint8_t MessagesNum;      // Range 1 to 3
  uint8_t MessageNumber;    // Range 1 to 3
  uint8_t SatellitesInView;
  uint8_t SatelliteID[4];   // Range 1 to 32
  int Elevation[4];         // Max 90 (in degrees)
  int Azimuth[4];           // Range 0 to 359 (in degrees)
  int SNR[4];               // Range 0 to 99, null when not tracking
  uint8_t signalID;         /* NMEA-defined GNSS signal ID, see Signal Identifiers
                                table (only available in NMEA 4.10 and later)
                            */
  char checksum[6];
} GSV_t;
typedef struct GLL{
  char Header[10];
  char Latitude[20];        // ddmm.mmmm
  uint8_t NSIndicator;      // N=north or S=south
  char Longitude[20];       // ddmm.mmmm
  uint8_t EWIndicator;      // E=east or W=west
  char UTCtime[15];         // hhmmss.sss
  char Status;              // A=data valid or V=data not valid
  char Mode;                /* A=Autonomous, D=DGPS, E=DR
                                Only present in NMEA version 3.00 or later
                            */
  char checksum[6];
} GLL_t;
typedef struct RMC{
  char Header[10];
  char UTCtime[12];         // hhmmss.sss
  char Status;              // A=data valid or V=data not valid
  char Latitude[20];        // ddmm.mmmm
  uint8_t NSIndicator;         // N=north or S=south
  char Longitude[20];       // ddmm.mmmm
  uint8_t EWIndicator;         // E=east or W=west
  float SpeedOverGround;    // In Knots
  float CourseOverGround;   // In Degrees
  char Date[12];            // ddmmyy
  float MagneticVariation;  // Magnetic Variation (degrees)
  char VariationSense;      // E=East & W=West
  char Mode;                /* A=Autonomous, D=DGPS, E=DR
                                Only present in NMEA version 2.3 or later
                            */
  char NavStatus;           /* Navigational status indicator: V (Equipment is not
                              providing navigational status information, fixed field,
                              only available in NMEA 4.10 and later)
                            */
  char checksum[6];
} RMC_t;
typedef struct GGA{
  char Header[10];
  char UTCtime[12];         // hhmmss.sss
  char Latitude[20];        // ddmm.mmmm
  uint8_t NSIndicator;      // N=north or S=south
  char Longitude[20];       // ddmm.mmmm
  uint8_t EWIndicator;      // E=east or W=west
  uint8_t FixQuality;       /*  0   : Fix not available or invalid
                                1   : GPS SPS Mode, fix valid
                                2   : Differential GPS, SPS mode, fix valid
                                3-5 : Not supported
                                6   : Dead Reckoning Mode, fix valid
                            */
  uint8_t SatellitesInView; // Range 0 to 12
  float HDOP;               // Horizontal Dilution of Precision
  float MSLAltitude;        // In meters
  char AltitudeUnits;       // In meters
  float GeoidSeparation;    // In meters
  char GeoidSeparationUnits;// In meters
  float AgeOfDGPSData;      // Null fields when DGPS is not used (In seconds)
  uint8_t DGPSStationID;
  char checksum[6];
} GGA_t;
typedef struct MSS{
  char Header[10];
  uint8_t SignalStrength;       // SS of tracked frequency (In dB)
  uint8_t SignalToNoiseRatio;   // SNR of tracked frequency (In dB)
  float BeaconFrequency;        // Currently tracked frequency (In kHz)
  uint8_t BeaconBitRate;        // Bits per second
  uint8_t ChannelNumber;        /* The channel of the beacon being used if a 
                                    multi-channel beacon receiver is used
                                    !!! Only present in NMEA version 2.3 or later
                                */
  char checksum[6];
} MSS_t;
typedef struct ZDA{
  char Header[10];
  char UTCtime[12];             // hhmmss.sss
  uint8_t Day;                  // 01 to 31
  uint8_t Month;                // 01 to 12
  uint16_t Year;                // 1980 to 2079
  uint8_t LocalZoneHours;       // Offset from UTC (set to 00)
  uint8_t LocalZoneMinutes;     // Offset from UTC (set to 00)
  char checksum[6];
} ZDA_t;
typedef struct VTG{
  char Header[10];
  float Course;             // Measured heading (degrees true)
  char Reference;           // T: track made good is relative to true north
  char MagneticCourse;      // Track made good (degrees magnetic)
  char MagneticReference;   // M: track made good is relative to magnetic north
  float SpeedKnots;         // Measured horizontal speed (In knots)
  char SpeedUnits;          // N: speed is measured in knots
  float SpeedOverGround;    // Speed over ground in kilometers/hour (kph)
  char SpeedOverGroundUnits;// K: speed over ground is measured in kph
  char Mode;                /* A = Autonomous, D = DGPS, E = DR
                                Only applies to NMEA version 2.3 or later
                            */
  char checksum[6];;
} VTG_t;
GSA_t GSA_Handler(const char* data){
  GSA_t sat_data;

  char* cursor = data;
  strcpy(sat_data.checksum,(strrchr(data, '*')));
  // Serial.print(F("Checksum : "));
  // Serial.println(sat_data.checksum);

  for(int i = 0 ; i < 18 ; i++ ){
    char Field[12] = "-1";
    memset(Field+2, '\0', 8);
    

    char *delimit = ",*\r\n";
    int span = strcspn(cursor, delimit);
    
    if (span) memmove(Field,cursor,span);

    cursor+=span+1;

    // Serial.print(F("["));
    // Serial.print(i);
    // Serial.print(F("] : Span : "));
    // Serial.print(span);
    // Serial.print(F(" | Field : "));
    // Serial.print(Field);
    // Serial.print(F(" | Cursor : "));
    // Serial.print(cursor);


    if(i == 0){
      strcpy(sat_data.Header,Field);
    }else if (i == 1){
      sat_data.Mode1 = Field[0];
    }else if (i == 2){
      sat_data.Mode2 = atoi(Field);
    }else if (i >= 3 && i <= 14){
      sat_data.PRN[i-3] = atoi(Field);
    }else if (i == 15){
      sat_data.PDOP = atof(Field);
    }else if (i == 16){
      sat_data.HDOP = atof(Field);
    }else if (i == 17){
      sat_data.VDOP = atof(Field);
    }
  }
  return sat_data;
}
GSV_t GSV_Handler(const char *data){
  GSV_t sat_data;
  for (int i = 0 ; i < 4 ; i++ ){
    sat_data.SatelliteID[i] = 0;
    sat_data.Elevation[i] = 0;
    sat_data.Azimuth[i] = 0;
    sat_data.SNR[i] = 0 ;
  }
  
  char* cursor = data;
  strcpy(sat_data.checksum,(strrchr(data, '*')));
  // Serial.print(F("Checksum : "));
  // Serial.println(sat_data.checksum);

  uint8_t fields=0;
  for (int i = 0 ; i < strcspn(data, '*');i++)
    if(data[i]==',' || data[i] =='*')  fields++;

  for(int i = 0 ; i < fields ; i++ ){
    char Field[12] = "-1";
    memset(Field+2, '\0', 8);
    
    char *delimit = ",*\r\n";
    int span = strcspn(cursor, delimit);
    
    if (span){ memmove(Field,cursor,span); Field[span]='\0';}

    cursor+=span+1;

    Serial.print(F("["));
    Serial.print(i);
    Serial.print(F("] : Span : "));
    Serial.print(span);
    Serial.print(F(" | Field : "));
    Serial.print(Field);
    Serial.print(F(" | Cursor : "));
    Serial.println(cursor);


    if(i == 0){
      strcpy(sat_data.Header,Field);
    } else if(i == 1){
      sat_data.MessagesNum = atoi(Field);
    } else if (i == 2){
      sat_data.MessageNumber = atoi(Field);
    } else if (i == 3){
      sat_data.SatellitesInView = atoi(Field);
    } else if (i == fields){
      sat_data.signalID = atoi(Field);
    } else if (i%4 == 0){
      sat_data.SatelliteID[(i/4)-1]=atoi(Field);
    } else if ((i-1)%4 == 0){
      sat_data.Elevation[((i-1)/4)-1] = atoi(Field);
    } else if ((i-2)%4 == 0){
      sat_data.Azimuth[((i-2)/4)-1] = atoi(Field);
    } else if ((i-3)%4 == 0){
      sat_data.SNR[((i-3)/4)-1] = atoi(Field);
    }
  }
  return sat_data;
}
GLL_t GLL_Handler(char *data){
  GLL_t sat_data;
  

  char* cursor = data;
  strcpy(sat_data.checksum,(strrchr(data, '*')));
  // Serial.print(F("Checksum : "));
  // Serial.println(sat_data.checksum);

  for(int i = 0 ; i < 8 ; i++ ){
    char Field[20] = "-";
    memset(Field+1, '\0', 19);
    

    char *delimit = ",*\r\n";
    int span = strcspn(cursor, delimit);
    
    if (span) memmove(Field,cursor,span);

    cursor+=span+1;

    // Serial.print(F("["));
    // Serial.print(i);
    // Serial.print(F("] : Span : "));
    // Serial.print(span);
    // Serial.print(F(" | Field : "));
    // Serial.print(Field);
    // Serial.print(F(" | Cursor : "));
    // Serial.print(cursor);


    if(i == 0){
      strcpy(sat_data.Header,Field);
    }else if(i == 1){
      strcpy(sat_data.Latitude,Field);
    }else if (i == 2){
      sat_data.NSIndicator = Field[0];
    }else if (i == 3){
      strcpy(sat_data.Longitude,Field);
    }else if (i == 4){
      sat_data.EWIndicator = Field[0];
    } else if (i == 5){
      strcpy(sat_data.UTCtime,Field);
    } else if (i == 6){
      sat_data.Status = Field[0];
    } else if (i == 7){
      sat_data.Mode = Field[0];
    }
  }
  return sat_data;
}
RMC_t RMC_Handler(char *data){
  RMC_t sat_data;
  
  char* cursor = data;
  strcpy(sat_data.checksum,(strrchr(data, '*')));
  // Serial.print(F("Checksum : "));
  // Serial.println(sat_data.checksum);

  for(int i = 0 ; i < 14 ; i++ ){
    char Field[20] = "-";
    memset(Field+1, '\0', 19);
    

    char *delimit = ",*\r\n";
    int span = strcspn(cursor, delimit);
    
    if(span) memmove(Field,cursor,span);

    cursor+=span+1;

    // Serial.print(F("["));
    // Serial.print(i);
    // Serial.print(F("] : Span : "));
    // Serial.print(span);
    // Serial.print(F(" | Field : "));
    // Serial.print(Field);
    // Serial.print(F(" | Cursor : "));
    // Serial.print(cursor);


    if(i == 0){
      strcpy(sat_data.Header,Field);
    }else if(i == 1){
      strcpy(sat_data.UTCtime,Field);
    }else if (i == 2){
      sat_data.Status = Field[0];
    }else if (i == 3){
      strcpy(sat_data.Latitude,Field);
    }else if (i == 4){
      sat_data.NSIndicator = Field[0];
    } else if (i == 5){
      strcpy(sat_data.Longitude,Field);
    } else if (i == 6){
      sat_data.EWIndicator = Field[0];
    } else if (i == 7){
      sat_data.SpeedOverGround = atof(Field);
    } else if (i == 8){
      sat_data.CourseOverGround = atof(Field);
    } else if (i == 9){
      strcpy(sat_data.Date,Field);
    } else if (i == 10){
      sat_data.MagneticVariation = atof(Field);
    }else if (i == 11){
      sat_data.VariationSense = Field[0];
    } else if (i == 12){
      sat_data.Mode = Field[0];
    } else if (i == 13){
      sat_data.NavStatus = Field[0];
    }
  }
  return sat_data;
}
GGA_t GGA_Handler(char *data){
  GGA_t sat_data;
  

  char* cursor = data;
  strcpy(sat_data.checksum,(strrchr(data, '*')));
  // Serial.print(F("Checksum : "));
  // Serial.println(sat_data.checksum);

  for(int i = 0 ; i < 15 ; i++ ){
    char Field[20] = "-";
    memset(Field+1, '\0', 19);
    

    char *delimit = ",*\r\n";
    int span = strcspn(cursor, delimit);
    
    if(span) memmove(Field,cursor,span);

    cursor+=span+1;

    // Serial.print(F("["));
    // Serial.print(i);
    // Serial.print(F("] : Span : "));
    // Serial.print(span);
    // Serial.print(F(" | Field : "));
    // Serial.print(Field);
    // Serial.print(F(" | Cursor : "));
    // Serial.print(cursor);

    if(i == 0){
      strcpy(sat_data.Header,Field);
    }else if(i == 1){
      strcpy(sat_data.UTCtime,Field);
    }else if (i == 2){
      strcpy(sat_data.Latitude,Field);
    }else if (i == 3){
      sat_data.NSIndicator = Field[0];
    } else if (i == 4){
      strcpy(sat_data.Longitude,Field);
    } else if (i == 5){
      sat_data.EWIndicator = Field[0];
    } else if (i == 6){
      sat_data.FixQuality = atoi(Field);
    } else if (i == 7){
      sat_data.SatellitesInView = atoi(Field);
    } else if (i == 8){
      sat_data.HDOP = atof(Field);
    } else if (i == 9){
      sat_data.MSLAltitude = atof(Field);
    } else if (i == 10){
      sat_data.AltitudeUnits = Field[0];
    } else if (i == 11){
      sat_data.GeoidSeparation = atof(Field);
    } else if (i == 12){
      sat_data.GeoidSeparationUnits = Field[0];
    } else if (i == 13){
      sat_data.AgeOfDGPSData = atof(Field);
    } else if (i == 14){
      sat_data.DGPSStationID = atoi(Field);
    }
  }
  return sat_data;
}
MSS_t MSS_Handler(char *data){
 MSS_t sat_data;
  

  char* cursor = data;
  strcpy(sat_data.checksum,(strrchr(data, '*')));
  // Serial.print(F("Checksum : "));
  // Serial.println(sat_data.checksum);

  for(int i = 0 ; i < 6 ; i++ ){
    char Field[20] = "-";
    memset(Field+1, '\0', 19);
    

    char *delimit = ",*\r\n";
    int span = strcspn(cursor, delimit);
    
    if(span) memmove(Field,cursor,span);

    cursor+=span+1;

    // Serial.print(F("["));
    // Serial.print(i);
    // Serial.print(F("] : Span : "));
    // Serial.print(span);
    // Serial.print(F(" | Field : "));
    // Serial.print(Field);
    // Serial.print(F(" | Cursor : "));
    // Serial.print(cursor);

    if(i == 0){
      strcpy(sat_data.Header,Field);
    } else if(i == 1){
      sat_data.SignalStrength = atoi(Field);
    } else if (i == 2){
      sat_data.SignalToNoiseRatio = atoi(Field);
    } else if (i == 3){
      sat_data.BeaconFrequency = atof(Field);
    } else if (i == 4){
      sat_data.BeaconBitRate = atoi(Field);
    } else if (i == 5){
      sat_data.ChannelNumber = atoi(Field);
    }
  }
  return sat_data;
}
ZDA_t ZDA_Handler(char *data){
  ZDA_t sat_data;
  

  char* cursor = data;
  strcpy(sat_data.checksum,(strrchr(data, '*')));
  // Serial.print(F("Checksum : "));
  // Serial.println(sat_data.checksum);

  for(int i = 0 ; i < 7 ; i++ ){
    char Field[20] = "-";
    memset(Field+1, '\0', 19);
    

    char *delimit = ",*\r\n";
    int span = strcspn(cursor, delimit);
    
    if(span) memmove(Field,cursor,span);

    cursor+=span+1;

    // Serial.print(F("["));
    // Serial.print(i);
    // Serial.print(F("] : Span : "));
    // Serial.print(span);
    // Serial.print(F(" | Field : "));
    // Serial.print(Field);
    // Serial.print(F(" | Cursor : "));
    // Serial.print(cursor);


    if(i == 0){
      strcpy(sat_data.Header,Field);
    }else if(i == 1){
      strcpy(sat_data.UTCtime,Field);
    }else if (i == 2){
      sat_data.Day = atoi(Field);
    }else if (i == 3){
      sat_data.Month = atoi(Field);
    } else if (i == 4){
      sat_data.Year = atoi(Field);
    } else if (i == 5){
      sat_data.LocalZoneHours = atoi(Field);
    } else if (i == 6){
      sat_data.LocalZoneMinutes = atoi(Field);
    }
  }
  return sat_data;
}
VTG_t VTG_Handler(char *data){
  VTG_t sat_data;
  

  char* cursor = data;
  strcpy(sat_data.checksum,(strrchr(data, '*')));
  // Serial.print(F("Checksum : "));
  // Serial.println(sat_data.checksum);

  for(int i = 0 ; i < 10 ; i++ ){
    char Field[20] = "-";
    memset(Field+1, '\0', 19);
    

    char *delimit = ",*\r\n";
    int span = strcspn(cursor, delimit);
    
    if(span) memmove(Field,cursor,span);

    cursor+=span+1;

    // Serial.print(F("["));
    // Serial.print(i);
    // Serial.print(F("] : Span : "));
    // Serial.print(span);
    // Serial.print(F(" | Field : "));
    // Serial.print(Field);
    // Serial.print(F(" | Cursor : "));
    // Serial.print(cursor);

    if(i == 0){
      strcpy(sat_data.Header,Field);
    }else if(i == 1){
      sat_data.Course = atof(Field);
    }else if (i == 2){
      sat_data.Reference = Field[0];
    }else if (i == 3){
      sat_data.MagneticCourse = atof(Field);
    } else if (i == 4){
      sat_data.MagneticReference = Field[0];
    } else if (i == 5){
      sat_data.SpeedKnots = atof(Field);
    } else if (i == 6){
      sat_data.SpeedUnits = Field[0];
    } else if (i == 7){
      sat_data.SpeedOverGround = atof(Field);
    } else if (i == 8){
      sat_data.SpeedOverGroundUnits = Field[0];
    } else if (i == 9){
      sat_data.Mode = Field[0];
    }
  }
  return sat_data;
}

typedef union message_format{
  GSA_t GSA;
  GSV_t GSV;
  GLL_t GLL;
  RMC_t RMC;
  GGA_t GGA;
  MSS_t MSS;
  ZDA_t ZDA;
  VTG_t VTG;
} message_t;

typedef struct packet{
  message_t message;
  frmt_t format;
  // (void*)print();
} packet_t;



packet_t read(const char* data){
  String buf = data;

  buf.remove(0,3);

  packet_t pack;

  if (buf.startsWith("GSA")){
    pack.message.GSA = GSA_Handler(data);
    pack.format = GSA;
  } else if (buf.startsWith("GSV")){
    pack.message.GSV = GSV_Handler(data);
    pack.format = GSV;
  } else if (buf.startsWith("GLL")){
    pack.message.GLL = GLL_Handler(data);
    pack.format = GLL;
  } else if (buf.startsWith("RMC")){
    pack.message.RMC = RMC_Handler(data);
    pack.format = RMC;
  } else if (buf.startsWith("GGA")){
    pack.message.GGA= GGA_Handler(data);
    pack.format = GGA;
  } else if (buf.startsWith("MSS")){
    pack.message.MSS = MSS_Handler(data);
    pack.format = MSS;
  } else if (buf.startsWith("ZDA")){
    pack.message.ZDA = ZDA_Handler(data);
    pack.format = ZDA;
  } else if (buf.startsWith("VTG")){
    pack.message.VTG = VTG_Handler(data);
    pack.format = VTG;
  } else {

  }
  return pack;
}

void SerialPrintMessage(packet_t message){
  Serial.flush();
  if (message.format == GSA){
    GSA_t packet = message.message.GSA;

    Serial.print(F("Header : "));
    Serial.println(packet.Header);
    Serial.print(F("Mode 1 : "));
    Serial.println(packet.Mode1);
    Serial.print(F("Mode 2 : "));
    Serial.println(packet.Mode2);
    Serial.print(F("HDOP : "));
    Serial.println(packet.HDOP);
    Serial.print(F("PDOP : "));
    Serial.println(packet.PDOP);
    Serial.print(F("VDOP : "));
    Serial.println(packet.VDOP);
    for (int i = 0 ; i < 12 ; i++ ){
      Serial.print(F("PRN["));
      Serial.print(i);
      Serial.print(F("] : "));
      Serial.println(packet.PRN[i]);
    }
    Serial.print(F("CHECKSUM : "));
    Serial.println(packet.checksum);
  } else if (message.format == GSV){
    GSV_t packet = message.message.GSV;

    Serial.print(F("Header : "));
    Serial.println(packet.Header);
    Serial.print(F("Number of Messages : "));
    Serial.println(packet.MessagesNum);
    Serial.print(F("Message Number : "));
    Serial.println(packet.MessageNumber);
    Serial.print(F("Satellites in View  : "));
    Serial.println(packet.SatellitesInView);

    for (int i = 0 ; i < 4 ; i++ ){
      Serial.print(F("Satellite ID ["));
      Serial.print(i+1);
      Serial.print(F("] : "));
      Serial.println(packet.SatelliteID[i]);

      Serial.print(F("Elevation ["));
      Serial.print(i+1);
      Serial.print(F("] : "));
      Serial.println(packet.Elevation[i]);

      Serial.print(F("Azimuth ["));
      Serial.print(i+1);
      Serial.print(F("] : "));
      Serial.println(packet.Azimuth[i]);
      
      Serial.print(F("SNR ["));
      Serial.print(i+1);
      Serial.print(F("] : "));
      Serial.println(packet.SNR[i]);
    }
    Serial.print(F("SignalID : "));
    Serial.println(packet.signalID);
    Serial.print(F("CHECKSUM : "));
    Serial.println(packet.checksum);
  } else if (message.format == GLL){
    GLL_t packet = message.message.GLL;

    Serial.print(F("Header : "));
    Serial.println(packet.Header);
    Serial.print(F("Latitude : "));
    Serial.println(packet.Latitude);
    Serial.print(F("N/S Indicator : "));
    Serial.println((char)packet.NSIndicator);
    Serial.print(F("Longitude : "));
    Serial.println(packet.Longitude);
    Serial.print(F("E/W Indicator : "));
    Serial.println((char)packet.EWIndicator);
    Serial.print(F("UTC Time : "));
    Serial.println(packet.UTCtime);
    Serial.print(F("Status : "));
    Serial.println(packet.Status);
    Serial.print(F("Mode : "));
    Serial.println(packet.Mode);
    Serial.print(F("Checksum : "));
    Serial.println(packet.checksum);
  } else if (message.format == RMC){
    RMC_t packet = message.message.RMC;

    Serial.print(F("Header : "));
    Serial.println(packet.Header);
    Serial.print(F("UTC Time : "));
    Serial.println(packet.UTCtime);
    Serial.print(F("Status : "));
    Serial.println(packet.Status);
    Serial.print(F("Latitude : "));
    Serial.println(packet.Latitude);
    Serial.print(F("N/S Indicator  : "));
    Serial.println((char)packet.NSIndicator);
    Serial.print(F("Longitude : "));
    Serial.println(packet.Longitude);
    Serial.print(F("E/W Indicator  : "));
    Serial.println((char)packet.EWIndicator);
    Serial.print(F("Speed Over Ground : "));
    Serial.println(packet.SpeedOverGround);
    Serial.print(F("Course Over Ground : "));
    Serial.println(packet.CourseOverGround);
    Serial.print(F("Date : "));
    Serial.println(packet.Date);
    Serial.print(F("Magnetic Variation : "));
    Serial.println(packet.MagneticVariation);
    Serial.print(F("East/West Indicator : "));
    Serial.println(packet.VariationSense);
    Serial.print(F("Mode : "));
    Serial.println(packet.Mode);
    Serial.print(F("NavStatus : "));
    Serial.println(packet.NavStatus);
    Serial.print(F("Checksum : "));
    Serial.println(packet.checksum);
  } else if (message.format == GGA){
    GGA_t packet = message.message.GGA;

    Serial.print(F("Header : "));
    Serial.println(packet.Header);
    Serial.print(F("UTC Time : "));
    Serial.println(packet.UTCtime);
    Serial.print(F("Latitude : "));
    Serial.println(packet.Latitude);
    Serial.print(F("N/S Indicator  : "));
    Serial.println((char)packet.NSIndicator);
    Serial.print(F("Longitude : "));
    Serial.println(packet.Longitude);
    Serial.print(F("E/W Indicator  : "));
    Serial.println((char)packet.EWIndicator);
    Serial.print(F("Position Fix Indicator  : "));
    Serial.println(packet.FixQuality);
    Serial.print(F("Satellites Used : "));
    Serial.println(packet.SatellitesInView);
    Serial.print(F("HDOP : "));
    Serial.println(packet.HDOP);
    Serial.print(F("MSL Altitude : "));
    Serial.println(packet.MSLAltitude);
    Serial.print(F("Units : "));
    Serial.println(packet.AltitudeUnits);
    Serial.print(F("Geoid Separation : "));
    Serial.println(packet.GeoidSeparation);
    Serial.print(F("Units : "));
    Serial.println(packet.GeoidSeparationUnits);
    Serial.print(F("Age of Diff. Corr.  : "));
    Serial.println(packet.AgeOfDGPSData);
    Serial.print(F("Diff. Ref. Station ID  : "));
    Serial.println(packet.DGPSStationID);
    Serial.print(F("Checksum : "));
    Serial.println(packet.checksum);
  } else if (message.format == MSS){
    MSS_t packet = message.message.MSS;

    Serial.print(F("Header : "));
    Serial.println(packet.Header);
    Serial.print(F("Signal Strength : "));
    Serial.println(packet.SignalStrength);
    Serial.print(F("Signal-to-Noise Ratio : "));
    Serial.println(packet.SignalToNoiseRatio);
    Serial.print(F("Beacon Frequency : "));
    Serial.println(packet.BeaconFrequency);
    Serial.print(F("Beacon Bit Rate : "));
    Serial.println(packet.BeaconBitRate);
    Serial.print(F("Channel Number : "));
    Serial.println(packet.ChannelNumber);
    Serial.print(F("Checksum : "));
    Serial.println(packet.checksum);
  } else if (message.format == ZDA){
    ZDA_t packet = message.message.ZDA;

    Serial.print(F("Header : "));
    Serial.println(packet.Header);
    Serial.print(F("UTC time : "));
    Serial.println(packet.UTCtime);
    Serial.print(F("Day : "));
    Serial.println(packet.Day);
    Serial.print(F("Month : "));
    Serial.println(packet.Month);
    Serial.print(F("Year : "));
    Serial.println(packet.Year);
    Serial.print(F("Local zone hour : "));
    Serial.println(packet.LocalZoneHours);
    Serial.print(F("Local zone minutes  : "));
    Serial.println(packet.LocalZoneMinutes);
    Serial.print(F("Checksum : "));
    Serial.println(packet.checksum);
  } else if (message.format == VTG){
    VTG_t packet = message.message.VTG;

    Serial.print(F("Header : "));
    Serial.println(packet.Header);
    Serial.print(F("Course  : "));
    Serial.println(packet.Course);
    Serial.print(F("Reference : "));
    Serial.println(packet.Reference);
    Serial.print(F("Magnetic Course : "));
    Serial.println(packet.MagneticCourse);
    Serial.print(F("Magnetic Reference : "));
    Serial.println(packet.MagneticReference);
    Serial.print(F("Speed on knots: "));
    Serial.println(packet.SpeedKnots);
    Serial.print(F("Units : "));
    Serial.println(packet.SpeedUnits);
    Serial.print(F("Speed Over Ground: "));
    Serial.println(packet.SpeedOverGround);
    Serial.print(F("Units : "));
    Serial.println(packet.SpeedOverGroundUnits);
    Serial.print(F("Mode : "));
    Serial.println(packet.Mode);
    Serial.print(F("Checksum : "));
    Serial.println(packet.checksum);
  }
}


