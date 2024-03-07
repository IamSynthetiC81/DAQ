#include "NMEA_Parser.h"

void setup() {  
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);

  Serial.flush();
  Serial.begin(115200);         //Arduino serial monitor thru USB cable 
  // Serial1.begin(9600);        // Serial1 port connected to GPS
  Serial.println(F("So it begins ...\n\t!King Theoden of Rohan"));  
  

  // delay(100);

  digitalWrite(LED_BUILTIN,HIGH);

  char* GSA_String = "$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,5*37\r\n";
  char* GSV_String = "$GPGSV,1,1,00,0*65\r\n";
  char* GLL_String = "$GNGLL,,,,,,V,N*7A\r\n";
  char* GLL_String_2 = "$GPGLL,3953.88008971,N,10506.75318910,W,034138.00,A,D*7A\r\n";
  char* RMC_String = "$GNRMC,,V,,,,,,,,,,N,V*37\r\n";
  char* RMC_String_2 = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
  char* RMC_String_3 = "$GPRMC,161229.487,A,3723.2475,N,12158.3416,W,0.13,309.62,120598, ,*10\r\n";
  char* RMC_String_4 = "$GPRMC,210230,A,3855.4487,N,09446.0071,W,0.0,076.2,130495,003.8,E*69\r\n";
  char* GGA_String = "$GNGGA,,,,,,0,00,99.99,,,,,,*56\r\n";
  char* GGA_String_2 = "$GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0 0031*4F\r\n";
  char* GGA_String_3 = "$GPGGA,002153.000,3342.6618,N,11751.3858,W,1,10,1.2,27.0,M,-34.2,M,,0000*5E\r\n";
  char* MSS_String = "$GPGSV,1,1,00,0*65\r\n";
  char* MSS_String_2 = "$GPMSS,55,27,318.0,100,1,*57\r\n";
  char* MSS_String_3 = "$GPMSS,26.4,7.2,283.5,200,2*61\r\n";
  char* ZDA_String = "$GPGSV,1,1,00,0*65\r\n";
  char* ZDA_String_2 = "$GPZDA,181813,14,10,2003,00,00*4F\r\n";
  char* ZDA_String_3 = "$GPZDA,172809.456,12,07,1996,00,00*45\r\n";
  char* VTG_String = "$GNVTG,,,,,,,,,N*2E\r\n";
  char* VTG_String_2 = "$GPVTG,140.88,T,,M,8.04,N,14.89,K,D*05\r\n";
  char* VTG_String_3 = "$GPVTG,309.62,T, ,M,0.13,N,0.2,K,A*23\r\n";

  packet_t packet;

  // Serial.println(F"(\t\t|\tGSA Packet Test\t|\n\t\t|\t---------------\t|"));
  // packet = read(GSA_String);
  // SerialPrintMessage(packet);
  

  // Serial.println(F("\t\t|\tGSV Packet Test\t|\n\t\t|\t---------------\t|"));
  // packet = read(GSV_String);
  // SerialPrintMessage(packet);

  // Serial.println(F("\t\t|\tGLL Packet Test\t|\n\t\t|\t---------------\t|"));
  // Serial.flush();

  // packet = read(GLL_String);
  // SerialPrintMessage(packet);
  
  // Serial.println(F("\t\t|\tGLL Packet Test 2\t|\n\t\t|\t---------------\t|"));
  // Serial.flush();

  // packet = read(GLL_String_2);
  // SerialPrintMessage(packet);

  // Serial.println(F("\t\t|\tRMC Packet Test\t|\n\t\t|\t---------------\t|"));
  // packet = read(RMC_String);
  // SerialPrintMessage(packet);

  // Serial.println(F("\t\t|\tRMC Packet Test 2\t|\n\t\t|\t---------------\t|"));
  // packet = read(RMC_String_2);
  // SerialPrintMessage(packet);

  // Serial.println(F("\t\t|\tRMC Packet Test 3\t|\n\t\t|\t---------------\t|"));
  // packet = read(RMC_String_3);
  // SerialPrintMessage(packet);

  // Serial.println(F("\t\t|\tRMC Packet Test 4\t|\n\t\t|\t---------------\t|"));
  // packet = read(RMC_String_4);
  // SerialPrintMessage(packet);

  //  Serial.println(F("\t\t|\tGGA Packet Test\t|\n\t\t|\t---------------\t|"));
  // packet = read(GGA_String);
  // SerialPrintMessage(packet);


  // Serial.println(F("\t\t|\tGGA Packet Test 2\t|\n\t\t|\t---------------\t|"));
  // packet = read(GGA_String_2);
  // SerialPrintMessage(packet);

  // Serial.println(F("\t\t|\tGGA Packet Test 3\t|\n\t\t|\t---------------\t|"));
  // packet = read(GGA_String_3);
  // SerialPrintMessage(packet);


  // Serial.println(F("\t\t|\tMSS Packet Test\t|\n\t\t|\t---------------\t|"));
  // packet = read(MSS_String);
  // SerialPrintMessage(packet);

  // Serial.println(F("\t\t|\tMSS Packet Test 2\t|\n\t\t|\t---------------\t|"));
  // packet = read(MSS_String_2);
  // SerialPrintMessage(packet);

  // Serial.println(F("\t\t|\tMSS Packet Test 3\t|\n\t\t|\t---------------\t|"));
  // packet = read(MSS_String_3);
  // SerialPrintMessage(packet);


  //  Serial.println(F("\t\t|\tZDA Packet Test\t|\n\t\t|\t---------------\t|"));
  // packet = read(ZDA_String);
  // SerialPrintMessage(packet);

  // Serial.println(F("\t\t|\tZDA Packet Test 2\t|\n\t\t|\t---------------\t|"));
  // packet = read(ZDA_String_2);
  // SerialPrintMessage(packet);

  // Serial.println(F("\t\t|\tZDA Packet Test 3\t|\n\t\t|\t---------------\t|"));
  // packet = read(ZDA_String_3);
  // SerialPrintMessage(packet);

  // Serial.println(F("\t\t|\tVTG Packet Test\t|\n\t\t|\t---------------\t|"));
  // packet = read(VTG_String);
  // SerialPrintMessage(packet);

  // Serial.println(F("\t\t|\tVTG Packet Test\t|\n\t\t|\t---------------\t|"));
  // packet = read(VTG_String_2);
  // SerialPrintMessage(packet);

  // Serial.println(F("\t\t|\tVTG Packet Test\t|\n\t\t|\t---------------\t|"));
  // packet = read(VTG_String_3);
  // SerialPrintMessage(packet);

  pinMode(LED_BUILTIN,OUTPUT);
}



void loop() { 
  unsigned int bufsize = 256;
  char buf[bufsize];
  if(Serial.available()){
    String something = Serial.readStringUntil('\n');
    // Serial.print(something);
    something.toCharArray(buf,bufsize);
    // Serial.print(buf);
    SerialPrintMessage(read(buf));
  }
}


//   if (ReadString.startsWith("GSA")) {               // I picked this sentence, you can pick any of the other labels and rearrange/add sections as needed. 
//       Serial.println(ReadString);                   // display raw GLL data in Serial Monitor

//                                                     // This section gets repeated for each delimeted bit of data by looking for the commas
//                                                     // Find Lattitude is first in GLL sentence, other senetences have data in different order
//       int Pos=ReadString.indexOf(',');              // look for comma delimetrer
//       ReadString.remove(0, Pos+1);                  // Remove Pos+1 characters starting at index=0, this one strips off "$GPGLL" in my sentence
//       Pos=ReadString.indexOf(',');                  // looks for next comma delimetrer, which is now the first comma because I removed the first segment   
//       char Lat[Pos];                                // declare character array Lat with a size of the dbit of data
//       for (int i=0; i <= Pos-1; i++){               // load charcters into array
//         Lat[i]=ReadString.charAt(i);           
//       }   
//       Serial.print(Lat);                            // display raw latitude data in Serial Monitor, I'll use Lat again in a few lines for converting   
//                                                     // repeating with a different char array variable        
//                                                     // Get Lattitude North or South
//       ReadString.remove(0, Pos+1);               
//       Pos=ReadString.indexOf(',');    
//         char LatSide[Pos];                          //declare different variable name
//           for (int i=0; i <= Pos-1; i++){
//             LatSide[i]=ReadString.charAt(i);        //fill the array          
//             Serial.println(LatSide[i]);             //display N or S
//           }

//                                                     //convert the variable array Lat to degrees Google can use
//           float LatAsFloat = atof (Lat);            //atof converts the char array to a float type
//           float LatInDeg;
//            if(LatSide[0]==char(78)) {               //char(69) is decimal for the letter "N" in ascii chart   
//                LatInDeg= ConvertData(LatAsFloat);   //call the conversion funcion (see below) 
//            }
//            if(LatSide[0]==char(83)) {        //char(69) is decimal for the letter "S" in ascii chart   
//                LatInDeg= -( ConvertData(LatAsFloat));   //call the conversion funcion (see below) 
//            }
//            Serial.println(LatInDeg,15); //display value Google can use in Serial Monitor, set decimal point value high
// //repeating with a different char array variable               
//        //Get Longitude
//         ReadString.remove(0, Pos+1);               
//         Pos=ReadString.indexOf(',');    
//         char Longit[Pos];             //declare different variable name
//            for (int i=0; i <= Pos-1; i++){
//             Longit[i]=ReadString.charAt(i);      //fill the array  
//            }   
//             Serial.print(Longit);      //display raw longitude data in Serial Monitor      
// //repeating with a different char array variable 
//             //Get Longitude East or West
//         ReadString.remove(0, Pos+1);              
//         Pos=ReadString.indexOf(',');    
//         char LongitSide[Pos];         //declare different variable name
//            for (int i=0; i <= Pos-1; i++){
//             LongitSide[i]=ReadString.charAt(i);      //fill the array          
//             Serial.println(LongitSide[i]);        //display raw longitude data in Serial Monitor
//            }       
//            //convert to degrees Google can use  
//           float LongitAsFloat = atof (Longit);    //atof converts the char array to a float type
//           float LongInDeg;
//          if(LongitSide[0]==char(69)) {        //char(69) is decimal for the letter "E" in ascii chart
//                  LongInDeg=ConvertData(LongitAsFloat);   //call the conversion funcion (see below
//          }    
//          if(LongitSide[0]==char(87)) {         //char(87) is decimal for the letter "W" in ascii chart
//                  LongInDeg=-(ConvertData(LongitAsFloat)); //call the conversion funcion (see below
//          }             
//            Serial.println(LongInDeg,15);  //display value Google can use in Serial Monitor, set decimal point value high
// //repeating with a different char array variable 
//             //Get TimeStamp - GMT
//         ReadString.remove(0, Pos+1);                
//         Pos=ReadString.indexOf(',');    
//         char TimeStamp[Pos];          //declare different variable name
//            for (int i=0; i <= Pos-1; i++){
//             TimeStamp[i]=ReadString.charAt(i);         //fill the array     
//             }
//            Serial.print(TimeStamp);   //display raw longitude data in Serial Monitor, GMT
//            Serial.println(F("");       
//    }
// }

//Conversion function
float ConvertData(float RawDegrees)
{ 
  float RawAsFloat = RawDegrees; 
  int firstdigits = ((int)RawAsFloat)/100; // Get the first digits by turning f into an integer, then doing an integer divide by 100;
  float nexttwodigits = RawAsFloat - (float)(firstdigits*100);
  float Converted = (float)(firstdigits + nexttwodigits/60.0);
  return Converted;
}