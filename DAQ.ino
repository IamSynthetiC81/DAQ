#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <FreqMeasure.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "NMEA_Parser.h"
#include <math.h>
#include <string.h>

#define SERIAL_LOGGIN 1
#define SD_LOGGIN 0

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
 *  01. Bit 00: Timer Overflow
 *  02. Bit 01: Aquisition Timed-Up
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
  float accelerationX;
  float accelerationY;
  float accelerationZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float	brakePressure;
  float	tps;
  uint8_t potValue;
  float	SpeedGPS;
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
void SerialPrintLog(){
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
    Serial.print(__LOG.GPS_Valid);
    Serial.print(",");
    Serial.println(__LOG.elapsedTime);
}

MPU6050 mpu;
Quaternion q;                                                                 // [w, x, y, z]         quaternion container
VectorInt16 aa;                                                               // [x, y, z]            accel sensor measurements
VectorInt16 gy;                                                               // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;                                                           // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;                                                          // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;                                                          // [x, y, z]            gravity vector
uint8_t fifoBuffer[64];                                                       // FIFO storage buffer
float euler[3];                                                               // [psi, theta, phi]    Euler angle container

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
    
    Serial.print(F(" |--Initializing DMP"));                                // load and configure the DMP
    devStatus = mpu.dmpInitialize();
  } else
    Serial.println(F("MPU6050 connection failed"));
  
  if (devStatus == 0) {                                                       // make sure it worked (returns 0 if so)
   Serial.println(F("\t\t\tPASS")); 
    mpu.CalibrateAccel(15);                                                   // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(15);                                                    // 
    
    Serial.print(F("\n |--Enabling DMP:"));
    mpu.setDMPEnabled(true);
    Serial.println(F("\t\t\tPASS")); 

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
/**
 * @brief Reads the GPS data from the Serial1 port and parses the data to get the speed.
 * 
 * @note The speed is stored in the __LOG structure and the GPS_Valid flag is set to true, meaning the speed is updated from the GPS.
 * @see NMNEA_Parser.h
 * 
 */
void readGPS(){
  __LOG.GPS_Valid = false;
  while(Serial1.available()){
    int bufsize = 256;
    char buf[bufsize];
    String	something =	Serial1.readStringUntil('\n');	                      //	Read the GPS data

    if (!something.substring(3).startsWith("VTG")) return;                    // If the data is not VTG, return
    
    something.toCharArray(buf,bufsize);				                                // Convert the GPS data to char array
    packet_t packet = read(buf);                                              // Parse the GPS data
    __LOG.SpeedGPS = packet.message.VTG.SpeedOverGround;                      // Get the speed from the GPS data
    __LOG.GPS_Valid = true;                                                   // Mark the GPS data as valid
  }
}
/**
 * @brief Reads the data from the MPU6050 and stores the data in the __LOG structure. 
 */
void readMPU6050(){
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {                              // Get the Latest packet 
    mpu.dmpGetQuaternion(&q, fifoBuffer);                                     // Get the Quaternion
    mpu.dmpGetAccel(&aa, fifoBuffer);                                         // Get the Acceleration
    mpu.dmpGetGravity(&gravity, &q);                                          // Get the Gravity
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);                            // Get the Linear Acceleration
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);                      // Get the Linear Acceleration in World Frame (acceleration - gravity)
    
    __LOG.accelerationX = aaWorld.x;
    __LOG.accelerationY = aaWorld.y;
    __LOG.accelerationZ = aaWorld.z;

    mpu.dmpGetEuler(euler, &q);                                               // Get the Euler Angles
    
    __LOG.gyroX = euler[0] * 180 / M_PI;
    __LOG.gyroY = euler[1] * 180 / M_PI;
    __LOG.gyroZ = euler[2] * 180 / M_PI;
  }
}
/**
 * @brief Calibrates the ADC, reads brake pressure, throttle position sensor and potentiometer values and stores them in the __LOG structure.
 * @see CalibrateSupplyVoltage()
 */
void readVariousSensors(){
  CalibrateSupplyVoltage();                                                   // Calibrate the supply voltage
  __LOG.brakePressure = (ReadVoltage(A2)-0.5)*11.5;                           // Read the Brake Pressure
  __LOG.tps = ReadVoltage(A1)/SupplyVoltage;                                  // Read the Throttle Position Sensor
  __LOG.potValue	= ((analogRead(potPin)/1023.0)-0.5)*SteeringWheel_RotationalRange/2;  // Read the Potentiometer value
}
/**
 * @brief Writes the log values to the SD card in the format of a CSV file.
 */
void WriteToSD(){
  const PROGMEM uint8_t BufferSize = 256;
  char buffer[BufferSize];
    
  snprintf(buffer, 256, BufferSize, "%f,%f,%f,%f,%f,%f,%f,%f,%d,%f,%d,%lf\n",__LOG.accelerationX,__LOG.accelerationY,__LOG.accelerationZ,__LOG.gyroX,__LOG.gyroY,__LOG.gyroZ,__LOG.brakePressure,__LOG.tps,__LOG.potValue, __LOG.SpeedGPS, __LOG.GPS_Valid, __LOG.elapsedTime );
  
  dataFile.print(buffer);
  dataFile.flush();
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
}
ISR(TIMER1_COMPA_vect){
  if (WINDOW){
    ERROR = true;
    ErrorFlag |= 0x01;
    Serial.println(F("Timer Overflow"));
  }
  WINDOW = !WINDOW;
}

/**
 * @brief Run the data acquisition process. 
 * @note The data acquisition process includes reading the GPS data, reading the MPU6050 data, reading the various sensors and storing the data in the __LOG structure.
 * @see readGPS()
 * @see readMPU6050()
 * @see readVariousSensors()
 * @see __LOG
 * 
 * @note The data is stored in the __LOG structure and can be printed using SerialPrintLog() function. 
 */
void AquireData(){
  readGPS();											                                          // Wait	for	VTG	messsage and print speed in	serial.	 
  readMPU6050();									                                          // Read data from	MPU6050
  readVariousSensors();         
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
  if (!SD.begin(10) && SD_LOGGIN) {                                           // SD card initialization
    Serial.println(F("SD initialization failed!\n\tFix and reboot to contrinue\n"));
    return;
  }

  dataFile = SD.open("data.txt", FILE_WRITE);                                 // Open the data file
  dataFile ? Serial.print(F("\t\t\tPASS")) : Serial.println(F("\t\t\tFAIL"));
  

  Serial.print(F("Opening Serial1 port at 9600 baud :"));
  Serial1.begin(9600);		                                                    // Serial1 port connected	to GPS
  Serial1 ? Serial.println(F("\tPASS ")) : Serial.println(F("FAIL"));

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