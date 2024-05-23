#include <avr/wdt.h>
#include <avr/iom2560.h>

#include <stdbool.h>

#include "src/MySerial/Serial.h"
#include <HardwareSerial.h>
#include <Arduino.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <u-blox_config_keys.h>
#include <u-blox_structs.h>

#include <I2Cdev.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include "MPU6050_6Axis_MotionApps612.h"

#include "src/ErrorHandler/src/ErrorHandler.h"
ErrorRegister ERROR_TIMER;
ErrorRegister ERROR_ADC;
ErrorRegister ERROR_IMU;
ErrorRegister ERROR_GPS;
ErrorRegister ERROR_SD;
ErrorRegister ERROR_SERIAL;

ErrorRegister errorRegisters[] = {
    ERROR_TIMER,
    ERROR_ADC,
    ERROR_IMU,
    ERROR_GPS,
    ERROR_SD,
    ERROR_SERIAL
};

ErrorHandler* errorHandler = new ErrorHandler(errorRegisters, 6);

static bool SERIAL_LOGGIN = false;
static bool SD_LOGGIN = false;
static bool PARALLEL_LOGGING = false;
static bool SERIAL_BRIDGE = false;

uint16_t TARGET_SAMPLING_RATE = 500;
#define BAUD_RATE 250000

/*  Button Declerations   */
#define PIN_START_BUTTON 2
#define PIN_STOP_BUTTON 3
#define PIN_RESET_BUTTON 21
#define PULSE_COUNTER 47

/*      OUTPUT PORT     */
#define OUTPUT_DATA_PORT PORTA
#define OUTPUT_ADDR_PORT PORTL
#define CLK_PIN 30

/*  Sensor Declerations */
#define	PIN_SUPPLY_SENSE A0
#define PIN_STEERING_WHEEL A1
#define PIN_BRAKE_PRESSURE A2
#define PIN_TROTTLE_POSITION_SENSOR A3
volatile uint8_t ADC_MUX_SELECT = 0x00;

/*          LED         */
#define LED LED_BUILTIN

static volatile bool SAMPLE_WINDOW = false;
static volatile bool ADC_WINDOW = false;
static volatile bool recording = false;
static volatile long StartTime = 0;


static bool IMU_present = false;
static bool SD_present = false;

static MPU6050 mpu;
static File dataFile;    
static SFE_UBLOX_GNSS myGNSS;

static HardwareSerial *LOG_SERIAL = &Serial;
static HardwareSerial *GPS_SERIAL = &Serial1;
static HardwareSerial *DBG_SERIAL = &Serial;

unsigned long __SAMPLING_RATE__ = TARGET_SAMPLING_RATE;

/**
 * @brief Structure to store the data
 * 
 * @param accelerationX Acceleration in X-axis. Divide according to the set max acceleration and multiply by G=9.81 to get the acceleration in m/s^2
 * @param accelerationY Acceleration in Y-axis. Divide according to the set max acceleration and multiply by G=9.81 to get the acceleration in m/s^2
 * @param accelerationZ Acceleration in Z-axis. Divide according to the set max acceleration and multiply by G=9.81 to get the acceleration in m/s^2
 * @param gyroX Gyroscope reading in X-axis. Multiply by 180/pi to get the value in degrees
 * @param gyroY Gyroscope reading in Y-axis. Multiply by 180/pi to get the value in degrees
 * @param gyroZ Gyroscope reading in Z-axis. Multiply by 180/pi to get the value in degrees
 * @param brakePressure Brake Pressure in bar ???
 * @param tps Throttle Position Sensor value. Divide by 1023 to get the value between 0 and 1
 * @param potValue Potentiometer value. Divide by 1023 to get the value between -0.5 and 0.5
 * @param SpeedGPS Speed from GPS in m/s
 * @param GPS_Valid GPS Validity
 * @param elapsedTime Elapsed time in milliseconds
 * 
 * @note The values are stored in the structure and can be printed using SerialPrintLog() function* 
 * @see SerialPrintLog() 
 */
typedef	struct information{
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
  long elapsedTime;
} log_t;

/**
 * @brief LOG structure to store the data
 */
log_t __LOG;

void LogToBuffer(uint8_t* buffer){
  buffer[0] = __LOG.accelerationX;
  buffer[1] = __LOG.accelerationX >> 8;
  buffer[2]=  __LOG.accelerationY;
  buffer[3]=  __LOG.accelerationY >> 8;
  buffer[4] = __LOG.accelerationZ;
  buffer[5] = __LOG.accelerationZ >> 8;
  buffer[6] = __LOG.gyroX;
  buffer[7] = __LOG.gyroX >> 8;
  buffer[8] = __LOG.gyroY;
  buffer[9] = __LOG.gyroY >> 8;
  buffer[10] = __LOG.gyroZ;
  buffer[11] = __LOG.gyroZ >> 8;
  buffer[12] = __LOG.Vref;
  buffer[13] = __LOG.Vref >> 8;
  buffer[14] = __LOG.BrakePressure;
  buffer[15] = __LOG.BrakePressure >> 8;
  buffer[16] = __LOG.ThrottlePositionSensor;
  buffer[17] = __LOG.ThrottlePositionSensor >> 8;
  buffer[18] = __LOG.SteeringWheel;
  buffer[19]=  __LOG.SteeringWheel >> 8;
  buffer[20]=  __LOG.CounterPulses;
  buffer[21]=  __LOG.CounterPulses >> 8;
  buffer[22]=  __LOG.GroundSpeed;
  buffer[23]=  __LOG.GroundSpeed >> 8;
  buffer[24]=  __LOG.GroundSpeed >> 16;
  buffer[25]=  __LOG.GroundSpeed >> 24;
  buffer[26]=  __LOG.GroundSpeed >> 32;
  buffer[27]=  __LOG.GroundSpeed >> 40;
  buffer[28]=  __LOG.GroundSpeed >> 48;
  buffer[29]=  __LOG.GroundSpeed >> 56;
  buffer[30]=  __LOG.Heading;
  buffer[31]=  __LOG.Heading >> 8;
  buffer[32]=  __LOG.Heading >> 16;
  buffer[33]=  __LOG.Heading >> 24;
  buffer[34]=  __LOG.Heading >> 32;
  buffer[35]=  __LOG.Heading >> 40;
  buffer[36]=  __LOG.Heading >> 48;
  buffer[37]=  __LOG.Heading >> 56;
  buffer[38]=  __LOG.Latitude;
  buffer[39]=  __LOG.Latitude >> 8;
  buffer[40]=  __LOG.Latitude >> 16;
  buffer[41]=  __LOG.Latitude >> 24;
  buffer[42]=  __LOG.Latitude >> 32;
  buffer[43]=  __LOG.Latitude >> 40;
  buffer[44]=  __LOG.Latitude >> 48;
  buffer[45]=  __LOG.Latitude >> 56;
  buffer[46]=  __LOG.Longitude;
  buffer[47]=  __LOG.Longitude >> 8;
  buffer[48]=  __LOG.Longitude >> 16;
  buffer[49]=  __LOG.Longitude >> 24;
  buffer[50]=  __LOG.Longitude >> 32;
  buffer[51]=  __LOG.Longitude >> 40;
  buffer[52]=  __LOG.Longitude >> 48;
  buffer[53]=  __LOG.Longitude >> 56;
  buffer[54]=  __LOG.Altitude;
  buffer[55]=  __LOG.Altitude >> 8;
  buffer[56]=  __LOG.Altitude >> 16;
  buffer[57]=  __LOG.Altitude >> 24;
  buffer[58]=  __LOG.Altitude >> 32;
  buffer[59]=  __LOG.Altitude >> 40;
  buffer[60]=  __LOG.Altitude >> 48;
  buffer[61]=  __LOG.Altitude >> 56;
  buffer[62]=  __LOG.elapsedTime;
  buffer[63]=  __LOG.elapsedTime >> 8;
  buffer[64]=  __LOG.elapsedTime >> 16;
  buffer[65]=  __LOG.elapsedTime >> 24;
  buffer[66]=  __LOG.elapsedTime >> 32;
  buffer[67]=  __LOG.elapsedTime >> 40;
  buffer[68]=  __LOG.elapsedTime >> 48;
  buffer[69]=  __LOG.elapsedTime >> 56;
}

void LogToCharArray(char *buffer){

  sprintf(buffer, "%d | %d | %d | %d | %d | %d\n",__LOG.accelerationX,__LOG.accelerationY,__LOG.accelerationZ,__LOG.gyroX,__LOG.gyroY,__LOG.gyroZ);
  // int16_t accelerationX;
  // int16_t accelerationY;
  // int16_t accelerationZ;
  // int16_t gyroX;
  // int16_t gyroY;
  // int16_t gyroZ;
  // uint16_t Vref;
  // uint16_t BrakePressure;
  // uint16_t ThrottlePositionSensor;
  // uint16_t SteeringWheel;
  // uint16_t CounterPulses;
  // uint64_t GroundSpeed;
  // uint64_t Heading;
  // uint64_t Latitude;
  // uint64_t Longitude;
  // uint64_t Altitude;
  // uint64_t elapsedTime;
}

inline void __attribute__ ((always_inline)) (*exportFunc)() = NULL;

inline void __attribute__ ((always_inline)) SerialPrintLog(){
  char buffer[512];
  sprintf(buffer, "%d | %d | %d | %d | %d | %d | %u | %u | %u | %u | %u | %ld | %ld | %ld | %ld | %ld | %ld \n\0",
  __LOG.accelerationX,
  __LOG.accelerationY,
  __LOG.accelerationZ,
  __LOG.gyroX,__LOG.gyroY,
  __LOG.gyroZ,
  __LOG.Vref,
  __LOG.BrakePressure,
  __LOG.ThrottlePositionSensor,
  __LOG.SteeringWheel,
  __LOG.CounterPulses,
  __LOG.GroundSpeed,
  __LOG.Heading,
  __LOG.Latitude,
  __LOG.Longitude,
  __LOG.Altitude,
  __LOG.elapsedTime
  );

  Serial.println(buffer);
}
inline void __attribute__ ((always_inline)) WriteToExternalMem(){
  uint8_t buffer[sizeof(log_t)];
  LogToBuffer(buffer);

  for (int i = 0 ; i < sizeof(buffer) ; i++ ){
    PORTA = buffer[i];
    // PORTA = (i % 2) == 0 ? 0x01 : 0x02;
    PORTL = 0x01;
    // delayMicroseconds(1);
    PORTL = 0x00;
    // delayMicroseconds(1);
  }
  PORTA = 0x00;
}
inline void __attribute__ ((always_inline)) VoidStream(){
  return;
}
inline void __attribute__ ((always_inline)) WriteToSD(){
  uint8_t buffer[sizeof(log_t)];
  LogToBuffer(buffer);

  for(int i = 0 ; i < sizeof(buffer); i++ ){
    dataFile.print(buffer[i]);
  }

  // dataFile.print(__LOG.accelerationX);
  // dataFile.print(",");
  // dataFile.print(__LOG.accelerationY);
  // dataFile.print(",");
  // dataFile.print(__LOG.accelerationZ);
  // dataFile.print(",");
  // dataFile.print(__LOG.gyroX);
  // dataFile.print(",");
  // dataFile.print(__LOG.gyroY);
  // dataFile.print(",");
  // dataFile.print(__LOG.gyroZ);
  // dataFile.print(",");
  // dataFile.print(__LOG.BrakePressure);
  // dataFile.print(",");
  // dataFile.print(__LOG.ThrottlePositionSensor);
  // dataFile.print(",");
  // dataFile.print(__LOG.SteeringWheel);
  // dataFile.print(",");
  // dataFile.print(__LOG.CounterPulses);
  // dataFile.print(",");
  // dataFile.print(__LOG.GroundSpeed);
  // dataFile.print(",");
  // dataFile.print(__LOG.Heading);
  // dataFile.print(",");
  // dataFile.print(__LOG.Latitude);
  // dataFile.print(",");
  // dataFile.print(__LOG.Longitude);
  // dataFile.print(",");
  // dataFile.print(__LOG.Altitude);
  // dataFile.print(",");
  // dataFile.println(__LOG.elapsedTime);    
}

void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct){

  /*            DEBUG             */
  // Serial.println(String("Latitude : ") + ubxDataStruct->lat);
  // Serial.println(String("Longtitude : ") + ubxDataStruct->lon);
  // Serial.println(String("MSL Height : ") + ubxDataStruct->hMSL);
  // Serial.println(String("GroundSpeed : ") + myGNSS.getGroundSpeed());
  // Serial.println(String("Heading : ") +  myGNSS.getHeading());


  if (ubxDataStruct->fixType != 0){ 
    __LOG.Latitude = ubxDataStruct->lat;
    __LOG.Longitude = ubxDataStruct->lon;
    __LOG.Altitude = ubxDataStruct->hMSL;
    __LOG.GroundSpeed = myGNSS.getGroundSpeed();
    __LOG.Heading = myGNSS.getHeading();

    /*            DEBUG             */
    // long latitude = myGNSS.getLatitude();
    // Serial.print(F("Lat: "));
    // Serial.print(latitude);

    // long longitude = myGNSS.getLongitude();
    // Serial.print(F(" Long: "));
    // Serial.print(longitude);

    // long speed = myGNSS.getGroundSpeed();
    // Serial.print(F(" Speed: "));
    // Serial.print(speed);
    // Serial.print(F(" (mm/s)"));

    // long heading = myGNSS.getHeading();
    // Serial.print(F(" Heading: "));
    // Serial.print(heading);
    // Serial.print(F(" (degrees * 10^-5)"));

    // int pDOP = myGNSS.getPDOP();
    // Serial.print(F(" pDOP: "));
    // Serial.println(pDOP / 100.0, 2); // Convert pDOP scaling from 0.01 to 1

  } else {
    __LOG.Latitude = 0xffffffff;
    __LOG.Longitude = 0xffffffff;
    __LOG.Altitude = 0xffffffff;
    __LOG.GroundSpeed = 0xffffffff;
    __LOG.Heading = 0xffffffff;
    Serial.println("No fix ...");
    delay(1000);

  }
}

bool initGPS(SFE_UBLOX_GNSS *myGNSS) {
  const PROGMEM char CONFIG[] = {
    // Disable NMEA
    0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
    0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
    0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
    0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
    0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
    0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off
    
    // Disable UBX
    0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC, //NAV-PVT off
    0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9, //NAV-POSLLH off
    0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0, //NAV-STATUS off

    // Enable UBX
    0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
    // 0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
    // 0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS onInit

    // Rate
    0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
    // 0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
    // 0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)

    0xB5, 0x62, 0x06, 0x08, 0x13, 0x00, 0x01, 0x01, 0x00, 0x00, 0x07, 0x00, 0x92, 0x20, 0x00, 0x02, 0x00, 0x92, 0x20, 0x07, 0x00, 0x91, 0x20, 0x07, 0xD8, 0x38, // Selects correct packages

    0xB5,0x62,0x06,0x8A,0x0C,0x00,0x01,0x01,0x00,0x00,0x01,0x00,0x52,0x40,0x00,0xC2,0x01,0x00,0xF4,0xB1,
  };

  Serial1.begin(9600);

  Stream *gps;
  gps = &Serial1;

  if (myGNSS->begin(*gps, 1100, false) == false){
    Serial.println(F("GPS not detected\n"));
    return false;
  }


  // send configuration data in UBX protocol
  for(int i = 0; i < sizeof(CONFIG); i++) {                        
    Serial1.write( pgm_read_byte(CONFIG+i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }

  Serial1.setTimeout(100);

  // Serial1.flush();
  // Serial1.begin(115200);

  myGNSS->setUART1Output(COM_TYPE_UBX);
  myGNSS->saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  myGNSS->setAutoPVTcallbackPtr(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata
}

void SerialBridge(HardwareSerial* src, HardwareSerial* dst){
  if(src->available()) 
    dst->write(src->read());
  if (dst->available())
    src->write(dst->read());
}

void MPU6050_init(){
  uint8_t devStatus;                                                          // return status after each device operation (0 = PASS, !0 = error)
  uint16_t packetSize;                                                        // expected DMP packet size (default is 42 bytes)

  wdt_reset();

  Serial.print(F("Initializing I2C Drivers :"));
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE                            // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000);                                                    // 400kHz I2C clock. Comment this line if code is not compiling
    Serial.println(F("\t\tPASS"));
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    Serial.println(F("\t\tPASS"));
  #endif

  Wire.beginTransmission(0x68);                                               // Start I2C communication with the MPU6050
  // VEC[ERREG_GN1] |= (Wire.endTransmission() << PROTOCOL_I2C_STUCK_BUS) ;       // Check if the MPU6050 is connected
  // if (VEC[ERREG_GN1] & (1 << PROTOCOL_I2C_STUCK_BUS) != 0){
  //   Wire.end();
  //   pinMode(21, OUTPUT); // pin 21 is SCL
  //   digitalWrite(21, HIGH);
  //   delayMicroseconds(10);
  //   for (int i = 0; i < 10; i++){ // Send 10 clock pulses to free up I2C bus
  //     digitalWrite(21, LOW);
  //     delayMicroseconds(10);
  //     digitalWrite(21, HIGH);
  //     delayMicroseconds(10);
  //   }
  //   Wire.begin();
  // }

  Serial.print(F(" |--Initializing MPU6050"));
  mpu.initialize();
  Serial.println(F("\t\tPASS"));

  Serial.print(F(" |--Testing device connection:"));                         // verify connection
  if (mpu.testConnection()){
    Serial.println(F("\t\tPASS")); 
    IMU_present = true ;
    
    Serial.print(F(" |--Initializing DMP"));                                // load and configure the DMP
    devStatus = mpu.dmpInitialize();
  } else
    Serial.println(F("MPU6050 connection failed"));
  
  if (devStatus == 0) {                                                       // make sure it worked (returns 0 if so)

    wdt_reset();

    Serial.println(F(" |--Calibrating:"));                                    // Caibrate
    mpu.CalibrateAccel(15);                                                   // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(15);

    Serial.print(F("\n |--Enabling DMP:"));
    mpu.setDMPEnabled(true);
    Serial.println(F("\t\t\tPASS")); 

    packetSize = mpu.dmpGetFIFOPacketSize();                                  // get expected DMP packet size for later comparison

    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);                           // Set the accelerometer range to 4G
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}
bool initTimers(uint16_t  freq){
  cli();                                                                        // Disable all interrupts                  

  // Calculate the top value for Timer1
  unsigned int topValue = (F_CPU / (freq * 8)) - 1;

  // Set the Timer1 mode to CTC (Clear Timer on Compare Match)
  TCCR1A &= ~(1 << WGM10);
  TCCR1A &= ~(1 << WGM11);
  TCCR1B |= (1 << WGM12);
  TCCR1B &= ~(1 << WGM13);

  // Set the prescaler to 8
  TCCR1B &= ~(1 << CS10);
  TCCR1B |= (1 << CS11);
  TCCR1B &= ~(1 << CS12);

  // Set the top value
  OCR1A = topValue;

  /* Init timer 5 for use as a counter  */
  TCCR5A = 0;                                                                   // Init Timer3
  TCCR5B = 0;                                                                   // Init Timer3
  TIMSK5 = 0;                                                                   // Timer/Counter3 Interrupt Mask Register
  TCNT5 = 0;

  
  TCCR5A &= ~(1 << WGM50);                                                      // Normal
  TCCR5A &= ~(1 << WGM51);                                                      // Normal
  TCCR5B &= ~(1 << WGM51);                                                      // Normal
  TCCR5B &= ~(1 << WGM52);                                                      // Normal
  TCCR5B &= ~(1 << WGM53);                                                      // Normal

  TCCR5B &= ~(1 << ICES5);                                                      // Falling edge trigger
  
  TCCR5B |= (1 << CS52);                                                        // Set CS32 to 1
  TCCR5B |= (1 << CS51);                                                        // Set CS31 to 1          
  TCCR5B &= ~(1 << CS50);                                                       // Set CS30 to 0     

  // Enable the Input Capture Noise Canceler
  TCCR5B |= (1 << ICNC5);   

  sei();                                                                        // Enable all interrupts                                                                 

  return true;  
} 

inline void __attribute__ ((always_inline))  readGPS(){
  myGNSS.checkUblox();      // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks();  // Check if any callbacks are waiting to be processed.
}

/**
 * @brief Reads the data from the various sensors and stores it in the __LOG structure.
 * 
 * @note The data is stored in the __LOG structure and can be printed using SerialPrintLog() function
 * @see SerialPrintLog()
 * @see CalibrateSupplyVoltage()
 * 
*/
inline void __attribute__ ((always_inline))  readVariousSensors(){
  /* Math functions that need to b calculated for each sensor are the following :
   * brakePressure = (ReadVoltage(A2)-0.5)*11.5;                           
   * tps = ReadVoltage(A1)/SupplyVoltage;                                  
   * potValue	= ((analogRead(potPin)/1023.0)-0.5)*SteeringWheel_RotationalRange/2;  
  */

  __LOG.CounterPulses = TCNT5;                                                // Get the counter pulses

  // analogReference(INTERNAL1V1);				                                       // Choose 1.1V internal reference
  ADMUX = (ADMUX & 0x3F) | (1 << REFS0) | (1 << REFS1);                       // Set Vref to internal 1V1
  ADMUX = (ADMUX & 0xFC) | ADC_MUX_SELECT;                                    // Set the MUX to the first ADC pin

  // Start the conversion
  ADCSRA |= (1 << ADSC);  
}

/**
 * @brief Initializes the ADC
 */
inline void __attribute__ ((always_inline)) ADC_init(){
  ADMUX |= (1 << REFS0);                                                      // Set the reference voltage to AVCC
  
  ADCSRB = (ADCSRB & 0x70) | (1 << ADTS2) | (1 << ADTS1);                     // Sets Trigger source to Timer1 Overflow
  DIDR0 = 0xff;                                                               // Disable Login Inputs from Analog Pins
  DIDR2 = 0xff;                                                               // Disable Login Inputs from Analog Pins

  // set an interrupt for when the conversion is complete 
  ADCSRA |= (1 << ADIE);
  ADCSRA |= (1 << ADEN);                                                          // Enable the ADC
}

void ISR_startRecording(){
  recording = true;
  TIMSK1 |= (1 << OCIE1A);                                                    // Enable the Timer1 compare interrupt A
  StartTime = millis();
  digitalWrite(LED, HIGH);
}
void ISR_stopRecording(){
  recording = false;
  digitalWrite(LED, LOW);
  TIMSK1 &= B11011000;                                                        // Disable Timer1

  if(SD_present) dataFile.close();
}

/**
 * @brief When the compare match occurs in Timer1, the ISR sets the SAMPLE_WINDOW flag. When set, data  are sampled.
 * 
 * @note This is the interrupt service routine for the Timer1 compare match
 * @see initTimers()
 * 
 */
ISR(TIMER1_COMPA_vect){
  // if (SAMPLE_WINDOW)  VEC[ERREG_ARD] |= (1 << SAMPLING_RATE_LOW);
  SAMPLE_WINDOW = true;
}

ISR(ADC_vect){
  switch(ADC_MUX_SELECT){
    case 0x00:
      __LOG.Vref = ADC;
      analogReference(DEFAULT);					                                      // Choose the default reference
      break;
    case 0x01:
      __LOG.SteeringWheel = ADC;
      break;
    case 0x02:
      __LOG.BrakePressure = ADC;
      break;
    case 0x03:
      __LOG.ThrottlePositionSensor = ADC;
      break;
  } 
 
  // increment ADC_MUX_SELECT from 0x00 to 0x03 looping back when done
  ADC_MUX_SELECT = (ADC_MUX_SELECT + 1) & 0x03;

  ADMUX = (ADMUX & 0xF8) | ADC_MUX_SELECT;                                    // Set the MUX to the next ADC pin
  if (ADC_MUX_SELECT != 0x00){ 
    ADCSRA |= (1 << ADSC);                                                    // Start the conversion
  }
}


// ISR(USART0_RX_vect){

// }

#include "src/SERCOMM/src/SERCOMM.h"

char COMMAND_START[] = "Start";
char COMMAND_STOP[] = "Stop";
char COMMAND_RESET[] = "Reset";
char COMMAND_SERIAL_ENABLE[] = "EnableSerial";
char COMMAND_SERIAL_DISABLE[] = "DisableSerial";
char COMMAND_PARALLEL_ENABLE[] = "EnableParallel";
char COMMAND_PARALLEL_DISABLE[] = "DisableParallel";
char COMMAND_FREQUENCY_GET[] = "getSamplingFrequency";
char COMMAND_FREQUENCY_SET[] = "setSamplingFrequency";
char COMMAND_SD_ENABLE[] = "EnableSD";
char COMMAND_SD_DISABLE[] = "DisableSD";

void FUNC_START_SAMPLING(){
  recording = true;
  TIMSK1 |= (1 << OCIE1A);                                                    // Enable the Timer1 compare interrupt A
  StartTime = millis();

  // digitalWrite(LED_BUILTIN, HIGH);
  PORTB |=  (1 << PIN7);
}
void FUNC_STOP_SAMPLING(){
  recording = false;

  // digitalWrite(LED_BUILTIN, LOW);
  PORTB &= !(1 << PIN7);
}
void FUNC_RESET(){
  asm volatile ("  jmp 0");
}
void FUNC_ENABLE_SERIAL(){
  exportFunc = SerialPrintLog;
}
void FUNC_DISABLE_SERIAL(){
  exportFunc = NULL;
}
void FUNC_ENABLE_PARALLEL(){
  // exportFunc = ExportParallel;
}
void FUNC_DISABLE_PARALLEL(){
  exportFunc = NULL;
}
void FUNC_GET_FREQUENCY(){
  Serial.print("Sampling Frequency: ");
  Serial.println(TARGET_SAMPLING_RATE);
}
void FUNC_SET_FREQUENCY(const int argc, char *argv[]){
  TARGET_SAMPLING_RATE = atoi(argv[0]);

  initTimers(TARGET_SAMPLING_RATE);

  Serial.print("Sampling Frequency set to: ");
  Serial.println(TARGET_SAMPLING_RATE);
}
void FUNC_ENABLE_SD(){
  exportFunc = WriteToSD;
}
void FUNC_DISABLE_SD(){
  exportFunc = NULL;
}

command_t commands[] = {
  initCommand(FUNC_START_SAMPLING, COMMAND_START, "Starting sampling"),
  initCommand(FUNC_STOP_SAMPLING, COMMAND_STOP, "Stoping sampling"),
  initCommand(FUNC_RESET, COMMAND_RESET, "Resetting the device"),
  initCommand(FUNC_ENABLE_SERIAL, COMMAND_SERIAL_ENABLE, "Enabing serial communication"),
  initCommand(FUNC_DISABLE_SERIAL, COMMAND_SERIAL_DISABLE, "Disabling serial communication"),
  initCommand(FUNC_ENABLE_PARALLEL, COMMAND_PARALLEL_ENABLE, "Enabling parallel communication"),
  initCommand(FUNC_DISABLE_PARALLEL, COMMAND_PARALLEL_DISABLE, "Disabling parallel communication"),
  initCommand(FUNC_GET_FREQUENCY, COMMAND_FREQUENCY_GET, "Fetching sampling frequency"),
  initCommand(FUNC_SET_FREQUENCY, COMMAND_FREQUENCY_SET, "Setting sampling frequency"),
  initCommand(FUNC_ENABLE_SD, COMMAND_SD_ENABLE, "Enabling SD card"),
  initCommand(FUNC_DISABLE_SD, COMMAND_SD_DISABLE, "Disabling SD card")
};

SERCOMM SerialCommander(commands, (sizeof(commands)/sizeof(commands[0])),30);
MySerial srt(115200);


void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial && SERIAL_LOGGIN);

  Serial.println(F("\n\t!!-Initializing the DAQ System-!!"));

  if (TARGET_SAMPLING_RATE * sizeof(log_t) > BAUD_RATE/10 ){
    Serial.println(F("The baud rate is too low for the sampling rate"));
    Serial.println(F("Increase the baud rate or decrease the sampling rate"));

    Serial.println("Baud rate must be larger than : " + String(__SAMPLING_RATE__ * sizeof(log_t)*8) + '\n');
  }

  exportFunc = VoidStream;

  /* Pin declerations */
  Serial.print(F("Initializing pins "));
  pinMode(PIN_START_BUTTON, INPUT_PULLUP);
  pinMode(PIN_STOP_BUTTON, INPUT_PULLUP);
  pinMode(PIN_RESET_BUTTON, INPUT_PULLUP);
  pinMode(CLK_PIN, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED,HIGH);
  delay(500);
  digitalWrite(LED,LOW);
  Serial.println(F("\t\t\tPASS"));

  Serial.print(F("Initializing pins "));
  ADC_init();
  Serial.println(F("\t\t\tPASS"));

  Serial.print(F("Initializing Watchdog :"));
  wdt_enable(WDTO_8S);                                                        // Enable watchdog with a 4-second timeout
  Serial.println(F("\t\t\tPASS"));

  Serial.println(F("Initializing IMU :"));
  MPU6050_init();                                                             // Initialize the MPU6050

  wdt_reset();                                                                // Reset the watchdog timer

  Serial.print(F("\nInitializing SD card :"));
  if (!SD.begin(53) && SD_LOGGIN) {                                           // SD card initialization
    // ERROR = true;
    // ErrorFlag = 0x05;
    Serial.println(F("SD initialization failed!\n\tFix and reboot to contrinue\n"));
    return;
  }

  wdt_reset();                                                                // Reset the watchdog timer

  dataFile = SD.open("data.txt", FILE_WRITE);                                 // Open the data file
  dataFile ? Serial.println(F("\t\t\tPASS")) : Serial.println(F("\t\t\t|--FAIL--|"));

  wdt_reset();                                                                // Reset the watchdog timer

  Serial.print(F("Initializing GPS :"));
  initGPS(&myGNSS);
  Serial.println("\t\t\tPASS");

  wdt_reset();                                                                // Reset the watchdog timer

  Serial.print(F("Setting up the interrupts :"));
  attachInterrupt(digitalPinToInterrupt(PIN_START_BUTTON), ISR_startRecording, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_STOP_BUTTON), ISR_stopRecording, FALLING);
  Serial.println(F("\t\tPASS "));

  wdt_reset();                                                                // Reset the watchdog timer

  Serial.print(F("Initializing Timers :"));
  if (initTimers(__SAMPLING_RATE__)){
    Serial.println(F("\t\t\tPASS"));
  } else {
    // ERROR = true;
    // ErrorFlag = 0x01;
    Serial.println(F("\t\t|--FAIL--|"));
  }



  // while(ERROR){};

  wdt_reset();                                                                // Reset the watchdog timer

  Serial.println(F("\n\t!!-Initialization complete-!!"));
  Serial.flush();
}

void loop()	{
  wdt_reset();                                                                // Reset the watchdog timer

  if(Serial.available()){
    Serial.readStringUntil('\n');
  }

  if (SERIAL_BRIDGE){
    SerialBridge(&Serial1,&Serial);
  }

  // if (!IMU_present) return;
  // if (!SD_present) return;
  if	(recording)	{
    if (!SAMPLE_WINDOW) return;
    
    digitalWrite(46,HIGH);
    readGPS();			                                                          // Wait	for	VTG	messsage and print speed in	Serial.	 
    // if(Serial.available()){
      
    //   char buffer[256];
    //   size_t size = Serial.readBytesUntil('\n', buffer , 256);
    //   SERCOMM_handler(buffer);
    // }
    
    
    /*  Read data from	MPU6050 */
    mpu.getMotion6(&__LOG.accelerationX, &__LOG.accelerationY, &__LOG.accelerationZ, &__LOG.gyroX, &__LOG.gyroY, &__LOG.gyroZ);
     
    readVariousSensors();
    
    exportFunc();                                                               // Export data

    __LOG.elapsedTime = micros() - StartTime;                                   // Calculate the elapsed time   

    SAMPLE_WINDOW = false;                                                      // Close the "Window"
  
    digitalWrite(46,LOW);
  }
}

// function that sends data through serial using ATMEGA2560 registers
void MySerialPrint(char *str){
  
}