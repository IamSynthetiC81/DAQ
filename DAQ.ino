#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "UBLOXgps.h"
#include <FreqMeasure.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "NMEA_Parser.h"
#include <math.h>
#include <string.h>

#define SERIAL_LOGGIN true
#define SD_LOGGIN false

/**
 * @brief Sampling Frequency
 * 
 * @note The sampling frequency is set to 200Hz
 * @see initTimers()
*/
#define __SAMPLING_RATE__ 200

#define __GPS_SERIAL__ Serial1
#define __LOG_STREAM__ Serial


/*  Button Declerations */
#define PIN_START_BUTTON 2
#define PIN_STOP_BUTTON 3
#define PIN_RESET_BUTTON 21
#define	PIN_SUPPLY_SENSE A0
#define PIN_STEERING_WHEEL A1
#define PIN_BRAKE_PRESSURE A2
#define PIN_TROTTLE_POSITION_SENSOR A3
#define LED LED_BUILTIN

volatile bool recording = false;
volatile long StartTime = 0;

static bool IMU_present = false;
static bool SD_present = false;
static volatile bool ERROR = false;
/**
 * @brief Flag to indicate an error in the system
 *  01. Timer Overflow
 *  02. Aquisition Timed-Up
 *  03. DMP Init Fail
 *  04. IMU Init Fail
 *  05. SD  Init Fail
 *  06. GPS Fail Init
 */
static volatile uint8_t ErrorFlag = false;
static volatile bool WINDOW = false;

MPU6050 mpu;
File dataFile;    

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
  uint16_t accelerationX;
  uint16_t accelerationY;
  uint16_t accelerationZ;
  uint16_t gyroX;
  uint16_t gyroY;
  uint16_t gyroZ;
  uint16_t supplyVoltage;
  uint16_t brakePressure;
  uint16_t tps;
  uint16_t potValue;
  float	SpeedGPS;
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
  bool GPS_Valid;
  unsigned long	elapsedTime;
} log_t;

/**
 * @brief LOG structure to store the data
 */
log_t __LOG;

/**
 * @brief Prints the log values to the Serial Monitor according to the format.
 */
inline void __attribute__ ((always_inline))  SerialPrintLog(){
    const PROGMEM uint8_t BufferSize = 256;
    char buffer[BufferSize];

    Serial.print(__LOG.accelerationX,HEX);
    Serial.print(",");
    Serial.print(__LOG.accelerationY,HEX);
    Serial.print(",");
    Serial.print(__LOG.accelerationZ,HEX);
    Serial.print(",");
    Serial.print(__LOG.gyroX,HEX);
    Serial.print(",");
    Serial.print(__LOG.gyroY,HEX);
    Serial.print(",");
    Serial.print(__LOG.gyroZ,HEX);
    Serial.print(",");
    Serial.print(__LOG.brakePressure,HEX);
    Serial.print(",");
    Serial.print(__LOG.tps,HEX);
    Serial.print(",");
    Serial.print(__LOG.potValue,HEX);
    Serial.print(",");
    Serial.print(__LOG.supplyVoltage,HEX);
    Serial.print(",");
    Serial.print(__LOG.SpeedGPS);
    Serial.print(",");
    Serial.print(__LOG.Latitude);
    Serial.print(",");
    Serial.print(__LOG.NSIndicator);
    Serial.print(",");
    Serial.print(__LOG.Longitude);
    Serial.print(",");
    Serial.print(__LOG.EWIndicator);
    Serial.print(",");
    Serial.print(__LOG.FixQuality);
    Serial.print(",");
    Serial.print(__LOG.GPS_Valid);
    Serial.print(",");
    Serial.println(__LOG.elapsedTime);
}

void MPU6050_init(){
  uint8_t devStatus;                                                          // return status after each device operation (0 = PASS, !0 = error)
  uint16_t packetSize;                                                        // expected DMP packet size (default is 42 bytes)

  __LOG_STREAM__.print(F("Initializing I2C Drivers :"));
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE                            // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000);                                                    // 400kHz I2C clock. Comment this line if code is not compiling
    __LOG_STREAM__.println(F("\t\tPASS"));
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    __LOG_STREAM__.println(F("\t\tPASS"));
  #endif

  Wire.beginTransmission(0x68);                                               // Start I2C communication with the MPU6050
  byte error = Wire.endTransmission();                                        // Check if the MPU6050 is connected

  if (error != 0){
    Wire.end();
    pinMode(21, OUTPUT); // pin 21 is SCL
    digitalWrite(21, HIGH);
    delayMicroseconds(10);
    for (int i = 0; i < 10; i++){ // Send 10 clock pulses to free up I2C bus
      digitalWrite(21, LOW);
      delayMicroseconds(10);
      digitalWrite(21, HIGH);
      delayMicroseconds(10);
    }
    Wire.begin();
  }

  Serial.print(F(" |--Initializing MPU6050"));
  mpu.initialize();
  Serial.println(F("\t\tPASS"));

  Serial.print(F(" |--Testing device connection:"));                         // verify connection
  if (mpu.testConnection()){
    Serial.println(F("\t\tPASS")); 
    IMU_present = true ;
    
    // Serial.print(F(" |--Initializing DMP"));                                // load and configure the DMP
    // devStatus = mpu.dmpInitialize();
  } else
    Serial.println(F("MPU6050 connection failed"));
  
  if (devStatus == 0) {                                                       // make sure it worked (returns 0 if so)
    Serial.println(F(" |--Calibrating:"));                                    // Caibrate
    mpu.CalibrateAccel(15);                                                   // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(15);                                                    // 

    // Serial.print(F("\n |--Enabling DMP:"));
    // mpu.setDMPEnabled(true);
    // Serial.println(F("\t\t\tPASS")); 

    packetSize = mpu.dmpGetFIFOPacketSize();                                  // get expected DMP packet size for later comparison

    // mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);                           // Set the accelerometer range to 4G
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
  uint16_t prescaler;
  
  uint16_t period = 1000000/freq;
  uint16_t compare = 0;

  cli();                                                                        // Disable all interrupts                  

  TCCR1A = 0;                                                                   // Init Timer1
  TCCR1B = 0;                                                                   // Init Timer1
  TIMSK1 = 0;                                                                   // Timer/Counter1 Interrupt Mask Register
  TCNT1 = 0;     

  TCCR1B |= (1 << WGM12);                                                       // CTC

  if (period < 65536){
    prescaler = 1;
    compare = period;
  } else if (period < 65536*8){
    prescaler = 8;
    compare = period/8;
  } else if (period < 65536*64){
    prescaler = 64;
    compare = period/64;
  } else if (period < 65536*256){
    prescaler = 256;
    compare = period/256;
  } else if (period < 65536*1024){
    prescaler = 1024;
    compare = period/1024;
  } else return false;

  OCR1A = compare;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << OCIE1A);

  sei();                                                                        // Enable all interrupts                                                                 

  return true;  
} 

/**
 * 
*/
inline void __attribute__ ((always_inline))  readGPS(){
  __LOG.GPS_Valid = false;
  if(__GPS_SERIAL__.available()){
    int bufsize = 256;
    char buf[bufsize];
    packet_t packet;
    String	something =	Serial1.readStringUntil('\n').substring(3);	            //	Read the GPS data from the serial port

    something.toCharArray(buf,bufsize);				                                  // Convert the GPS data to char array
    packet= read(buf);                                                          // Parse the GPS data

    if (packet.format == VTG){                                                  // Check if the data is a VTG message
      __LOG.SpeedGPS = packet.message.VTG.SpeedOverGround;                      // Get the speed from the GPS data
      __LOG.GPS_Valid = true;                                                   // Mark the GPS data as valid
    }else if (packet.format == GGA){                                            // Check if the data is a GGA message
      strcpy(__LOG.Latitude, packet.message.GGA.Latitude);
      strcpy(__LOG.Longitude, packet.message.GGA.Longitude);
      __LOG.NSIndicator = packet.message.GGA.NSIndicator;
      __LOG.EWIndicator = packet.message.GGA.EWIndicator;
      __LOG.FixQuality = packet.message.GGA.FixQuality;

      // Serial.print("Latitude : ");
      // Serial.println(packet.message.GGA.Latitude);
      // Serial.print("Longitude : ");
      // Serial.println(packet.message.GGA.Longitude);
    } 
  }
}

/**
 * @brief Reads the data from the MPU6050 and stores it in the __LOG structure.
 * 
 * @note The data is stored in the __LOG structure and can be printed using SerialPrintLog() function
 * @see SerialPrintLog()
 * @see MPU6050_init()
*/
inline void __attribute__ ((always_inline))  readMPU6050(){
  mpu.getMotion6(&__LOG.accelerationX, &__LOG.accelerationY, &__LOG.accelerationZ, &__LOG.gyroX, &__LOG.gyroY, &__LOG.gyroZ);
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
  // __LOG.brakePressure = (ReadVoltage(A2)-0.5)*11.5;                           // Read the Brake Pressure
  // __LOG.tps = ReadVoltage(A1)/SupplyVoltage;                                  // Read the Throttle Position Sensor
  // __LOG.potValue	= ((analogRead(potPin)/1023.0)-0.5)*SteeringWheel_RotationalRange/2;  // Read the Potentiometer value

  analogReference(INTERNAL1V1);				                                        // Choose 1.1V internal reference
  __LOG.supplyVoltage = analogRead(PIN_SUPPLY_SENSE);                         // Read the supply voltage
  analogReference(DEFAULT);					                                          // Choose the default reference

  __LOG.brakePressure = analogRead(PIN_BRAKE_PRESSURE);                       // Read the Brake Pressure
  __LOG.tps = analogRead(PIN_TROTTLE_POSITION_SENSOR);                        // Read the Throttle Position Sensor
  __LOG.potValue = analogRead(PIN_STEERING_WHEEL);                            // Read the Steering Wheel Position
}
/**
 * @brief Writes the log values to the SD card in the format of a CSV file.
 */
inline void __attribute__ ((always_inline))  WriteToSD(){
  
  dataFile.print(__LOG.accelerationX);
  dataFile.print(",");
  dataFile.print(__LOG.accelerationY);
  dataFile.print(",");
  dataFile.print(__LOG.accelerationZ);
  dataFile.print(",");
  dataFile.print(__LOG.gyroX);
  dataFile.print(",");
  dataFile.print(__LOG.gyroY);
  dataFile.print(",");
  dataFile.print(__LOG.gyroZ);
  dataFile.print(",");
  dataFile.print(__LOG.brakePressure);
  dataFile.print(",");
  dataFile.print(__LOG.tps);
  dataFile.print(",");
  dataFile.print(__LOG.potValue);
  dataFile.print(",");
  dataFile.print(__LOG.SpeedGPS);
  dataFile.print(",");
  dataFile.print(__LOG.Latitude);
  dataFile.print(",");
  dataFile.print(__LOG.NSIndicator);
  dataFile.print(",");
  dataFile.print(__LOG.Longitude);
  dataFile.print(",");
  dataFile.print(__LOG.EWIndicator);
  dataFile.print(",");
  dataFile.print(__LOG.FixQuality);
  dataFile.print(",");
  dataFile.print(__LOG.GPS_Valid);
  dataFile.print(",");
  dataFile.println(__LOG.elapsedTime);
  dataFile.flush();
}

void ISR_startRecording(){
  recording = true;
  TIMSK1 |= B00000010;
  StartTime = millis();
  digitalWrite(LED, HIGH);
}
void ISR_stopRecording(){
  recording = false;
  digitalWrite(LED, LOW);
  TIMSK1 &= B11011000;                                                        // Disable Timer1

  if(SD_present) dataFile.close();
}
ISR(TIMER1_COMPA_vect){
  if (WINDOW){
    ERROR = true;
    ErrorFlag |= 0x01;
    // Serial.println("\n\t!!\tOVERFLOW\t!!\n");
  }
  WINDOW = !WINDOW;
}

void setup() {
  ERROR = false;                                                            // Clear ERROR flag

  __LOG_STREAM__.begin(115200);
  while (!Serial && SERIAL_LOGGIN);

  /* Pin declerations */
  __LOG_STREAM__.print(F("Initializing pins "));
  pinMode(PIN_START_BUTTON, INPUT_PULLUP);
  pinMode(PIN_STOP_BUTTON, INPUT_PULLUP);
  pinMode(PIN_RESET_BUTTON, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  digitalWrite(LED,HIGH);
  delay(500);
  digitalWrite(LED,LOW);
  __LOG_STREAM__.println(F("\t\t\tPASS"));

  __LOG_STREAM__.println(F("Initializing IMU :"));
  MPU6050_init();                                                             // Initialize the MPU6050
  
  Serial.print(F("\nInitializing SD card :"));
  if (!SD.begin(53) && SD_LOGGIN) {                                           // SD card initialization
    ERROR = true;
    ErrorFlag = 0x05;
    __LOG_STREAM__.println(F("SD initialization failed!\n\tFix and reboot to contrinue\n"));
    return;
  }

  dataFile = SD.open("data.txt", FILE_WRITE);                                 // Open the data file
  dataFile ? __LOG_STREAM__.println(F("\t\t\tPASS")) : __LOG_STREAM__.println(F("\t\t\tFAIL"));

  __LOG_STREAM__.print(F("Initializing GPS :"));
  initGPS();
  __LOG_STREAM__.println("\t\t\tPASS");

  __LOG_STREAM__.print(F("Initializing FreqMeasure :"));
  FreqMeasure.begin();                                                        // Initialize the FreqMeasure library
  __LOG_STREAM__.println(F("\t\tPASS"));

  __LOG_STREAM__.print(F("Setting up the interrupts :"));
  attachInterrupt(digitalPinToInterrupt(PIN_START_BUTTON), ISR_startRecording, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_STOP_BUTTON), ISR_stopRecording, FALLING);
  __LOG_STREAM__.println(F("\t\tPASS "));

  Serial.print(F("Initializing Timers :"));
  if (initTimers(__SAMPLING_RATE__)){
    __LOG_STREAM__.println(F("\t\tPASS"));
    __LOG_STREAM__.print(" |--Sampling Frequency : ");
  } else {
    ERROR = true;
    ErrorFlag = 0x01;
    __LOG_STREAM__.println(F("\t\tFAIL"));

  }

  while(ERROR){};

  __LOG_STREAM__.println(F("\n\t!!-Initialization complete-!!"));
}

void loop()	{
  // if (!IMU_present) return;
  // if (!SD_present) return;

  if	(recording)	{

    readGPS();			                                                          // Wait	for	VTG	messsage and print speed in	serial.	 

    if (!WINDOW) return;

    double sum = 0;
    int count = 0;

    // if(FreqMeasure.available()){
    //   sum =+FreqMeasure.read();
    //   count = count + 1;
    //   if (count > 30){
    //     float freq = FreqMeasure.countToFrequency(sum/count);
    //     sum = 0;
    //     count = 0;
    //   }
    // }
  
    readMPU6050();									                                          // Read data from	MPU6050
    readVariousSensors();
    
    if(SERIAL_LOGGIN) SerialPrintLog();                                         // Print the data
    if(SD_LOGGIN) WriteToSD();                                                  // Write the data to the SD card

    __LOG.elapsedTime = millis() - StartTime;                                   // Calculate the elapsed time   

    WINDOW = false;                                                             // Set the window flag to false
  }
}