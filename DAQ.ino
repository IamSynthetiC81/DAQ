#include <avr/wdt.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <u-blox_config_keys.h>
#include <u-blox_structs.h>

#include "I2Cdev.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include "MPU6050_6Axis_MotionApps612.h"
#include "ErrorHandler.h"
#include "NMEA_Parser.h"

static bool SERIAL_LOGGIN = false;
static bool SD_LOGGIN = false;
static bool PARALLEL_LOGGING = false;

#define TARGET_SAMPLING_RATE 500
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
volatile bool recording = false;
volatile long StartTime = 0;
volatile uint8_t FreqMem = 0;


static bool IMU_present = false;
static bool SD_present = false;

static MPU6050 mpu;
static File dataFile;    
static SFE_UBLOX_GNSS myGNSS;

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
  uint16_t accelerationX;
  uint16_t accelerationY;
  uint16_t accelerationZ;
  uint16_t gyroX;
  uint16_t gyroY;
  uint16_t gyroZ;
  uint16_t Vref;
  uint16_t BrakePressure;
  uint16_t ThrottlePositionSensor;
  uint16_t SteeringWheel;
  uint16_t CounterPulses;
  uint64_t GroundSpeed;
  uint64_t Heading;
  uint64_t Latitude;
  uint64_t Longitude;
  uint64_t Altitude;
  uint64_t elapsedTime;
} log_t;

/**
 * @brief LOG structure to store the data
 */
log_t __LOG;

void WriteToExternalMem(){
  uint8_t buffer[] = {
    __LOG.accelerationX,
    __LOG.accelerationX >> 8,
    __LOG.accelerationY,
    __LOG.accelerationY >> 8,
    __LOG.accelerationZ,
    __LOG.accelerationZ >> 8,
    __LOG.gyroX,
    __LOG.gyroX >> 8,
    __LOG.gyroY,
    __LOG.gyroY >> 8,
    __LOG.gyroZ,
    __LOG.gyroZ >> 8,
    __LOG.Vref,
    __LOG.Vref >> 8,
    __LOG.BrakePressure,
    __LOG.BrakePressure >> 8,
    __LOG.ThrottlePositionSensor,
    __LOG.ThrottlePositionSensor >> 8,
    __LOG.SteeringWheel,
    __LOG.SteeringWheel >> 8,
    __LOG.CounterPulses,
    __LOG.CounterPulses >> 8,
    __LOG.GroundSpeed,
    __LOG.GroundSpeed >> 8,
    __LOG.GroundSpeed >> 16,
    __LOG.GroundSpeed >> 24,
    __LOG.GroundSpeed >> 32,
    __LOG.GroundSpeed >> 40,
    __LOG.GroundSpeed >> 48,
    __LOG.GroundSpeed >> 56,
    __LOG.Heading,
    __LOG.Heading >> 8,
    __LOG.Heading >> 16,
    __LOG.Heading >> 24,
    __LOG.Heading >> 32,
    __LOG.Heading >> 40,
    __LOG.Heading >> 48,
    __LOG.Heading >> 56,
    __LOG.Latitude,
    __LOG.Latitude >> 8,
    __LOG.Latitude >> 16,
    __LOG.Latitude >> 24,
    __LOG.Latitude >> 32,
    __LOG.Latitude >> 40,
    __LOG.Latitude >> 48,
    __LOG.Latitude >> 56,
    __LOG.Longitude,
    __LOG.Longitude >> 8,
    __LOG.Longitude >> 16,
    __LOG.Longitude >> 24,
    __LOG.Longitude >> 32,
    __LOG.Longitude >> 40,
    __LOG.Longitude >> 48,
    __LOG.Longitude >> 56,
    __LOG.Altitude,
    __LOG.Altitude >> 8,
    __LOG.Altitude >> 16,
    __LOG.Altitude >> 24,
    __LOG.Altitude >> 32,
    __LOG.Altitude >> 40,
    __LOG.Altitude >> 48,
    __LOG.Altitude >> 56,
    __LOG.elapsedTime,
    __LOG.elapsedTime >> 8,
    __LOG.elapsedTime >> 16,
    __LOG.elapsedTime >> 24,
    __LOG.elapsedTime >> 32,
    __LOG.elapsedTime >> 40,
    __LOG.elapsedTime >> 48,
    __LOG.elapsedTime >> 56,
  };

  for (int i = 0 ; i < sizeof(buffer) ; i++ ){
    PORTA = buffer[i];
    // PORTA = (i % 2) == 0 ? 0x01 : 0x02;
    PORTL = 0x01;
    delayMicroseconds(1);
    PORTL = 0x00;
    delayMicroseconds(1);
  }
  PORTA = 0x00;
}

void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct){
  __LOG.Latitude = ubxDataStruct->lat;
  __LOG.Longitude = ubxDataStruct->lon;
  __LOG.Altitude = ubxDataStruct->hMSL;
  __LOG.GroundSpeed = myGNSS.getGroundSpeed();
  __LOG.Heading = myGNSS.getHeading();
}

bool initGPS(SFE_UBLOX_GNSS *myGNSS) {
  const PROGMEM char CONFIG[] = {
    // Disable NMEA
    0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
    0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
    // 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
    0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
    0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
    // 0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off
    0xB5,0x62,0x06,0x8A,0x0E,0x00,0x01,0x01,0x00,0x00,0xBB,0x00,0x91,0x20,0x01,0xB1,0x00,0x91,0x20,0x01,0x70,0xB6,
    // Disable UBX
    0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x17,0xDC, //NAV-PVT off
    0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9, //NAV-POSLLH off
    0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0, //NAV-STATUS off

    // Enable UBX
    // 0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
    // 0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x01,0x00,0x00,0x00,0x00,0x13,0xBE, //NAV-POSLLH on
    // 0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x01,0x00,0x00,0x00,0x00,0x14,0xC5, //NAV-STATUS on

    // Rate
    // 0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
    // 0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
    0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)

    // 0xB5,0x62,0x06,0x8A,0x0C,0x00,0x01,0x01,0x00,0x00,0x01,0x00,0x52,0x40,0x00,0xC2,0x01,0x00,0xF4,0xB1,
  };

  Serial1.begin(9600);

  Stream *gps; 
  gps = &Serial1;

  if (myGNSS->begin(*gps, 250, false) == false){
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

/**
 * @brief Prints the log values to the Serial Monitor according to the format.
 */
inline void __attribute__ ((always_inline))  SerialPrintLog(){
  uint8_t buffer[] = {
    __LOG.accelerationX,
    __LOG.accelerationX >> 8,
    __LOG.accelerationY,
    __LOG.accelerationY >> 8,
    __LOG.accelerationZ,
    __LOG.accelerationZ >> 8,
    __LOG.gyroX,
    __LOG.gyroX >> 8,
    __LOG.gyroY,
    __LOG.gyroY >> 8,
    __LOG.gyroZ,
    __LOG.gyroZ >> 8,
    __LOG.Vref,
    __LOG.Vref >> 8,
    __LOG.BrakePressure,
    __LOG.BrakePressure >> 8,
    __LOG.ThrottlePositionSensor,
    __LOG.ThrottlePositionSensor >> 8,
    __LOG.SteeringWheel,
    __LOG.SteeringWheel >> 8,
    __LOG.CounterPulses,
    __LOG.CounterPulses >> 8,
    __LOG.GroundSpeed,
    __LOG.GroundSpeed >> 8,
    __LOG.GroundSpeed >> 16,
    __LOG.GroundSpeed >> 24,
    __LOG.GroundSpeed >> 32,
    __LOG.GroundSpeed >> 40,
    __LOG.GroundSpeed >> 48,
    __LOG.GroundSpeed >> 56,
    __LOG.Heading,
    __LOG.Heading >> 8,
    __LOG.Heading >> 16,
    __LOG.Heading >> 24,
    __LOG.Heading >> 32,
    __LOG.Heading >> 40,
    __LOG.Heading >> 48,
    __LOG.Heading >> 56,
    __LOG.Latitude,
    __LOG.Latitude >> 8,
    __LOG.Latitude >> 16,
    __LOG.Latitude >> 24,
    __LOG.Latitude >> 32,
    __LOG.Latitude >> 40,
    __LOG.Latitude >> 48,
    __LOG.Latitude >> 56,
    __LOG.Longitude,
    __LOG.Longitude >> 8,
    __LOG.Longitude >> 16,
    __LOG.Longitude >> 24,
    __LOG.Longitude >> 32,
    __LOG.Longitude >> 40,
    __LOG.Longitude >> 48,
    __LOG.Longitude >> 56,
    __LOG.Altitude,
    __LOG.Altitude >> 8,
    __LOG.Altitude >> 16,
    __LOG.Altitude >> 24,
    __LOG.Altitude >> 32,
    __LOG.Altitude >> 40,
    __LOG.Altitude >> 48,
    __LOG.Altitude >> 56,
    __LOG.elapsedTime,
    __LOG.elapsedTime >> 8,
    __LOG.elapsedTime >> 16,
    __LOG.elapsedTime >> 24,
    __LOG.elapsedTime >> 32,
    __LOG.elapsedTime >> 40,
    __LOG.elapsedTime >> 48,
    __LOG.elapsedTime >> 56,
  };

  for(int i = 0 ; i < sizeof(buffer); i++ ){
    Serial.print(buffer[i]);
  }
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
  VEC[ERREG_GN1] |= (Wire.endTransmission() << PROTOCOL_I2C_STUCK_BUS) ;       // Check if the MPU6050 is connected

  if (VEC[ERREG_GN1] & (1 << PROTOCOL_I2C_STUCK_BUS) != 0){
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

    wdt_reset();

    Serial.println(F(" |--Calibrating:"));                                    // Caibrate
    mpu.CalibrateAccel(15);                                                   // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(15);

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
  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.
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
  dataFile.print(__LOG.BrakePressure);
  dataFile.print(",");
  dataFile.print(__LOG.ThrottlePositionSensor);
  dataFile.print(",");
  dataFile.print(__LOG.SteeringWheel);
  dataFile.print(",");
  dataFile.print(__LOG.CounterPulses);
  dataFile.print(",");
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


/**
 * @brief Initializes the ADC
 * 
 * @note The ADC is initialized with the prescaler set to 128 
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
  if (SAMPLE_WINDOW) {
    VEC[ERREG_ARD] |= (1 << SAMPLING_RATE_LOW);
    if (FreqMem++ >= 100){
      __SAMPLING_RATE__ -= __SAMPLING_RATE__*0.05;
      initTimers(__SAMPLING_RATE__);

      Serial.println("Sampling rate decreased at : " +String(__SAMPLING_RATE__));
    }
  }
  FreqMem -= (FreqMem > 0);
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

void setup() {
  clearError();

  Serial.begin(BAUD_RATE);
  while (!Serial && SERIAL_LOGGIN);

  Serial.println(F("\n\t!!-Initializing the DAQ System-!!"));

  if (TARGET_SAMPLING_RATE * sizeof(log_t) > BAUD_RATE/10 ){
    Serial.println(F("The baud rate is too low for the sampling rate"));
    Serial.println(F("Increase the baud rate or decrease the sampling rate"));

    Serial.println("Baud rate must be larger than : " + String(__SAMPLING_RATE__ * sizeof(log_t)*8) + '\n');
  }

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
    
    String incomingMessage = Serial.readStringUntil('\n');
    // incomingMessage.setCharAt(incomingMessage.indexOf('\r'),'');

    if (incomingMessage.equals("Start")){
      Serial.println("Starting");
      recording = true;
      TIMSK1 |= (1 << OCIE1A);                                                    // Enable the Timer1 compare interrupt A
      StartTime = millis();
      digitalWrite(LED, HIGH);
    } else if (incomingMessage.equals("Stop")){
      recording = false;
      digitalWrite(LED, LOW);
      Serial.println("Stoping");
    } else if (incomingMessage.equals("EnableSerial")){
      SERIAL_LOGGIN = true;
      Serial.println("Enabling Serial");
    } else if (incomingMessage.equals("DisableSerial")){
      SERIAL_LOGGIN = false;
      Serial.println("Disabling Serial");
    } else if (incomingMessage.equals("EnableParallel")){
      PARALLEL_LOGGING = true;
      Serial.println("Enabling Parallel interface");
    } else if (incomingMessage.equals("DisableParallel")){
      PARALLEL_LOGGING = false;
      Serial.println("Disabling Parallel interface");
    }
  }


  // if (!IMU_present) return;
  // if (!SD_present) return;
  if	(recording)	{
    if (!SAMPLE_WINDOW) return;
    
    digitalWrite(46,HIGH);
    readGPS();			                                                          // Wait	for	VTG	messsage and print speed in	Serial.	 
    
    /*  Read data from	MPU6050 */
    mpu.getMotion6(&__LOG.accelerationX, &__LOG.accelerationY, &__LOG.accelerationZ, &__LOG.gyroX, &__LOG.gyroY, &__LOG.gyroZ);
     
    readVariousSensors();
    
    if(SERIAL_LOGGIN) SerialPrintLog();                                         // Print the data
    if(SD_LOGGIN) WriteToSD();                                                  // Write the data to the SD card
    if(PARALLEL_LOGGING) WriteToExternalMem();

    __LOG.elapsedTime = micros() - StartTime;                                   // Calculate the elapsed time   

    SAMPLE_WINDOW = false;                                                      // Set the SAMPLE_ flag to false
  
    digitalWrite(46,LOW);
  }
}