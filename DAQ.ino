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

#define SERIAL_LOGGIN false
#define SD_LOGGIN false

static uint8_t SupplyVoltage = 5;
#define	SupplyVoltageSense_Ratio 5.99
#define	SupplyVoltageSense_Pin 0

#define Sampling_Rate_100
// #define Sampling_Rate_80
// #define Sampling_Rate_60
// #define Sampling_Rate_50
// #define Sampling_Rate_10


#define	ADC_bit_range 1023												                            // 10 bit ADC;
#define	ReadVoltage(Pin) (SupplyVoltage*analogRead(Pin)/ADC_bit_range)	      //	Reads Voltage
inline void __attribute__ ((always_inline)) CalibrateSupplyVoltage(){
  analogReference(INTERNAL1V1);				// Choose 1.1V internal reference
  SupplyVoltage	= SupplyVoltageSense_Ratio*ReadVoltage(SupplyVoltageSense_Pin);
  analogReference(DEFAULT);					// Choose the default reference
}

#define	SteeringWheel_RotationalRange 270
#define potPin A1

/*  Button Declerations */
#define startButtonPin 2
#define stopButtonPin 3
#define resetButtonPin 21
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
  float	brakePressure;
  float	tps;
  uint8_t potValue;
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

    Serial.print(__LOG.accelerationX);
    Serial.print(",");
    Serial.print(__LOG.accelerationY);
    Serial.print(",");
    Serial.print(__LOG.accelerationZ);
    Serial.print(",");
    Serial.print(__LOG.gyroX);
    Serial.print(",");
    Serial.print(__LOG.gyroY);
    Serial.print(",");
    Serial.print(__LOG.gyroZ);
    Serial.print(",");
    Serial.print(__LOG.brakePressure);
    Serial.print(",");
    Serial.print(__LOG.tps);
    Serial.print(",");
    Serial.print(__LOG.potValue);
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

MPU6050 mpu;
File dataFile;                                                                // File to store the data

void MPU6050_init(){
  uint8_t devStatus;                                                          // return status after each device operation (0 = PASS, !0 = error)
  uint16_t packetSize;                                                        // expected DMP packet size (default is 42 bytes)

  Serial.print(F(" |--Initializing I2C devices"));
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
   Serial.println(F("\t\t\tPASS")); 
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
void initTimers(){
  uint8_t sps = 0;
  
  cli();                                                                        // Disable all interrupts                  
  TCCR1A = 0;                                                                   // Init Timer1
  TCCR1B = 0;                                                                   // Init Timer1
  TCNT1 = 0;     

  TCCR1B |= (1 << WGM12);                                                       // CTC

  #ifdef Sampling_Rate_100
    OCR1A = 624;                                                                // 100 Hz (16000000/((624+1)*256))
    TCCR1B |= (1 << CS12);                                                      // Prescaler = 256
    sps = 100;
  #endif
  #ifdef Sampling_Rate_80
    OCR1A =  24999;                                                             // = 16000000 / (8 * 80) - 1 (must be <65536)
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);                          // Prescaler = 8
    sps = 80;
  #endif  
  #ifdef Sampling_Rate_60
    OCR1A =  33332;                                                             // = 16000000 / (8 * 60.00060000600006) - 1 (must be <65536)
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);                          // Prescaler = 8
    sps = 60;
  #endif  
  #ifdef Sampling_Rate_50
    OCR1A =  39999;                                                             // = 16000000 / (8 * 50) - 1 (must be <65536)
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);                          // Prescaler = 8
    sps = 50;
  #endif
  #ifdef Sampling_Rate_10
    OCR1A = 6249;                                                               // 10 Hz (16000000/((6249+1)*256))
    TCCR1B |= (1 << CS12);                                                      // Prescaler = 256 
    sps = 10;
  #endif   
                                                                   
  Serial.println(F("\t\t\tPASS "));
  Serial.print(" |--Sampling Frequency : ");
  Serial.print(sps);
  Serial.println(" sps");

  sei();                                                                        // Enable all interrupts                                                                 
} 
void initGPS(){
  const PROGMEM char COMMAND_SET_UPDATE_RATE_10Hz_RAM = "B5 62 06 8A 0A 00 01 01 00 00 01 00 21 30 64 00 52 C3";
  const PROGMEM char COMMAND_SET_UPDATE_RATE_10Hz_BBR = "B5 62 06 8A 0A 00 01 02 00 00 01 00 21 30 64 00 53 CC";
  const PROGMEM char COMMAND_SET_VTG_MESSAGE_ONLY_RAM = "B5 62 06 8A 09 00 01 01 00 00 01 00 74 10 00 20 BB";
  const PROGMEM char COMMAND_SET_VTG_MESSAGE_ONLY_BBR = "B5 62 06 8A 09 00 01 02 00 00 01 00 74 10 00 21 C3";
  const PROGMEM char COMMAND_set_BAUDE_RATE_115200_RAM = "B5 62 06 8A 0C 00 01 01 00 00 01 00 52 40 00 C2 01 00 F4 B1";
  const PROGMEM char COMMAND_set_BAUDE_RATE_115200_BBR = "B5 62 06 8A 0C 00 01 02 00 00 01 00 52 40 00 C2 01 00 F5 BC";

  HardwareSerial *log = &Serial;
  HardwareSerial *gps = &Serial1;



  // GPS_init(log, gps, 115200, 100);

  Serial1.begin(115200);

  // restoreDefaults();

//     Serial1.write(COMMAND_DISABLE_ALL_MESSAGES);
//     Serial.print(".");
//     waitForAck();

//     Serial1.write(COMMAND_ENABLE_VTG_MESSAGE);
//     Serial.print(".");
//     waitForAck();

//     Serial1.write(COMMAND_BAUDE_RATE_115200);
//     Serial.print(".");
//     waitForAck();

    
//     Serial1.flush();                                                          // wait for last transmitted data to be sent 
//     Serial1.begin(115200);                                                    // Start with new baud rate
//     while(Serial1.available()) Serial.read();                                 // empty  out possible garbage from input buffer

//     Serial1.write(COMMAND_UPDATE_RATE_10Hz);
//     Serial.print(".");
//     waitForAck();  
}

void waitForAck() {
  const PROGMEM uint8_t UBX_ACK_ACK_HEADER[] = {0xB5, 0x62, 0x05, 0x01};
  uint8_t buffer[10];
  uint8_t index = 0;

  while (Serial.available()) {                                                // Wait for data to be available on the serial port
    uint8_t byte = Serial.read();                                             // Read a byte from the serial port
    if (byte == UBX_ACK_ACK_HEADER[index]) {                                  // Check if the byte matches the expected header
      buffer[index++] = byte;                                                 // Byte matches, so add it to the buffer and increment the index
      if (index == sizeof(UBX_ACK_ACK_HEADER))                                // Check if we've received the complete header
        break;                                                                // We've received the complete header, so we can stop waiting
    } else {                                                                  // Byte doesn't match, so reset the index and buffer
      index = 0;
      memset(buffer, 0, sizeof(buffer));
    }
  }
}

/**
 * @brief Reads the GPS data from the Serial1 port and parses the data to get the speed.
 * 
 * @note The speed is stored in the __LOG structure and the GPS_Valid flag is set to true, meaning the speed is updated from the GPS.
 * @see NMNEA_Parser.h
 * 
 */
inline void __attribute__ ((always_inline))  readGPS(){
  __LOG.GPS_Valid = false;
  if(Serial1.available()){
    int bufsize = 256;
    char buf[bufsize];
    packet_t packet;
    String	something =	Serial1.readStringUntil('\n').substring(3);	                      //	Read the GPS data
    // something.toCharArray(buf,bufsize);				                                // Convert the GPS data to char array
    // packet= read(buf);  
    // SerialPrintMessage(packet);

    if (something.startsWith("VTG")){
      
      something.toCharArray(buf,bufsize);				                                // Convert the GPS data to char array
      packet= read(buf);                                                      // Parse the GPS data
      
      __LOG.SpeedGPS = packet.message.VTG.SpeedOverGround;                      // Get the speed from the GPS data
      __LOG.GPS_Valid = true;                                                   // Mark the GPS data as valid

      Serial.print("SpeedOverGround : ");
      Serial.println(__LOG.SpeedGPS);
    }else if (something.startsWith("GGA")){
      
      something.toCharArray(buf,bufsize);				                                // Convert the GPS data to char array
      packet= read(buf);     

      strcpy(__LOG.Latitude, packet.message.GGA.Latitude);
      strcpy(__LOG.Longitude, packet.message.GGA.Longitude);
      __LOG.NSIndicator = packet.message.GGA.NSIndicator;
      __LOG.EWIndicator = packet.message.GGA.EWIndicator;
      __LOG.FixQuality = packet.message.GGA.FixQuality;

      Serial.print("Latitude : ");
      Serial.println(packet.message.GGA.Latitude);
      Serial.print("Longitude : ");
      Serial.println(packet.message.GGA.Longitude);
    } 
  }
}
/**
 * @brief Reads the data from the MPU6050 and stores the data in the __LOG structure. 
 */
inline void __attribute__ ((always_inline))  readMPU6050(){
  mpu.getMotion6(&__LOG.accelerationX, &__LOG.accelerationY, &__LOG.accelerationZ, &__LOG.gyroX, &__LOG.gyroY, &__LOG.gyroZ);
}
/**
 * @brief Calibrates the ADC, reads brake pressure, throttle position sensor and potentiometer values and stores them in the __LOG structure.
 * @see CalibrateSupplyVoltage()
 */
inline void __attribute__ ((always_inline))  readVariousSensors(){
  CalibrateSupplyVoltage();                                                   // Calibrate the supply voltage
  __LOG.brakePressure = (ReadVoltage(A2)-0.5)*11.5;                           // Read the Brake Pressure
  __LOG.tps = ReadVoltage(A1)/SupplyVoltage;                                  // Read the Throttle Position Sensor
  __LOG.potValue	= ((analogRead(potPin)/1023.0)-0.5)*SteeringWheel_RotationalRange/2;  // Read the Potentiometer value
}
/**
 * @brief Writes the log values to the SD card in the format of a CSV file.
 */
inline void __attribute__ ((always_inline))  WriteToSD(){
Serial.print(__LOG.accelerationX);
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
    // Serial.println(ErrorFlag);
  }
  WINDOW = !WINDOW;
}

/**
 * @brief Run the data acquisition routine. 
 * @note The data acquisition process includes reading the GPS data, reading the MPU6050 data, reading the various sensors and storing the data in the __LOG structure.
 * @see readGPS()
 * @see readMPU6050()
 * @see readVariousSensors()
 * @see __LOG
 * 
 * @note The data is stored in the __LOG structure and can be printed using SerialPrintLog() function. 
 */
inline void __attribute__ ((always_inline))  AquireData(){
  
  unsigned long t1 ,t2,t3,t4;
  t1 = micros();

  readGPS();			                                                          // Wait	for	VTG	messsage and print speed in	serial.	 
  t2 = micros();
  readMPU6050();									                                          // Read data from	MPU6050
  t3 = micros();
  readVariousSensors();
  t4 = micros();

  // Serial.print(t2-t1);
  // Serial.print(",");
  // Serial.print(t3-t2);
  // Serial.print(",");
  // Serial.println(t4-t3);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && SERIAL_LOGGIN);

  /* Pin declerations */
  Serial.print(F("Initializing pins "));
  pinMode(startButtonPin, INPUT_PULLUP);
  pinMode(stopButtonPin, INPUT_PULLUP);
  pinMode(resetButtonPin, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  digitalWrite(LED,HIGH);
  delay(500);
  digitalWrite(LED,LOW);
  Serial.println(F("\t\t\tPASS"));

  Serial.print(F("Initializing I2C Drivers :"));
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE                            // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000);                                                    // 400kHz I2C clock. Comment this line if code is not compiling
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.println(F("\t\tPASS"));
  
  Serial.println(F("Initializing IMU :"));
  MPU6050_init();                                                             // Initialize the MPU6050
  
  Serial.print(F("\nInitializing SD card :"));
  if (!SD.begin(53) && SD_LOGGIN) {                                           // SD card initialization
    ERROR = true;
    ErrorFlag = 0x05;
    Serial.println(F("SD initialization failed!\n\tFix and reboot to contrinue\n"));
    return;
  }

  dataFile = SD.open("data.txt", FILE_WRITE);                                 // Open the data file
  dataFile ? Serial.println(F("\t\t\tPASS")) : Serial.println(F("\t\t\tFAIL"));
  

  // Serial.print(F("Opening Serial1 port at 9600 baud :"));
  // Serial1.begin(9600);                                                      // Serial1 port connected	to GPS
  // if(Serial1){
  //   Serial.println(F("\tPASS "));
  // } else {
  //   ERROR = true;
  //   ErrorFlag = 0x06;
    
  //   Serial.println(F("FAIL"));
  // }

  Serial.print(F("Initializing GPS :"));
  initGPS();
  Serial.println(F("\t\tPASS "));

  Serial.print(F("Initializing FreqMeasure :"));
  FreqMeasure.begin();                                                        // Initialize the FreqMeasure library
  Serial.println(F("\t\tPASS "));

  Serial.print(F("Setting up the interrupts :"));
  attachInterrupt(digitalPinToInterrupt(startButtonPin), ISR_startRecording, FALLING);
  attachInterrupt(digitalPinToInterrupt(stopButtonPin), ISR_stopRecording, FALLING);
  Serial.println(F("\t\tPASS "));

  Serial.print(F("Initializing Timers :"));
  initTimers();

  Serial.println(F("\n\t!!-Initialization complete-!!"));
}

void loop()	{
  // if (!IMU_present) return;
  // if (!SD_present) return;

  if	(recording)	{
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
  
    AquireData();                                                               // Aquire the data
    if(SERIAL_LOGGIN) SerialPrintLog();                                         // Print the data
    if(SD_LOGGIN) WriteToSD();                                                  // Write the data to the SD card

    __LOG.elapsedTime = millis() - StartTime;                                   // Calculate the elapsed time   

    WINDOW = false;                                                             // Set the window flag to false
  }
}