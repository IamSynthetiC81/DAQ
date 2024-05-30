#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <u-blox_config_keys.h>
#include <u-blox_structs.h>

#include <avr/wdt.h>

#include <I2Cdev.h>
#include <Wire.h>
#include <SPI.h>

#include "src/Definitions.h"

#include "src/IMU/IMU.h"
#include "src/SD/SD.h"

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


/*          LED         */
#define LED LED_BUILTIN

static IMU mpu;
// static File dataFile;    
static SFE_UBLOX_GNSS myGNSS;

unsigned long __SAMPLING_RATE__ = TARGET_SAMPLING_RATE;



void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct){

  /*            DEBUG             */
  // Serial.println(String("Latitude : ") + ubxDataStruct->lat);
  // Serial.println(String("Longtitude : ") + ubxDataStruct->lon);
  // Serial.println(String("MSL Height : ") + ubxDataStruct->hMSL);
  // Serial.println(String("GroundSpeed : ") + myGNSS.getGroundSpeed());
  // Serial.println(String("Heading : ") +  myGNSS.getHeading());


  if (ubxDataStruct->fixType != 0){ 
    packt.Latitude = ubxDataStruct->lat;
    packt.Longitude = ubxDataStruct->lon;
    packt.Altitude = ubxDataStruct->hMSL;
    packt.GroundSpeed = myGNSS.getGroundSpeed();
    packt.Heading = myGNSS.getHeading();

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
    Serial.println("GPS not detected\n");
    return false;
  }


  // send configuration data in UBX protocol
  for(int i = 0; i < sizeof(CONFIG); i++) {                        
    Serial1.write( pgm_read_byte(CONFIG+i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }

  Serial1.setTimeout(100);

  Serial1.flush();
  Serial1.begin(115200);

  myGNSS->setUART1Output(COM_TYPE_UBX);
  myGNSS->saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  myGNSS->setAutoPVTcallbackPtr(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata
}

inline void __attribute__ ((always_inline))  readGPS(){
  myGNSS.checkUblox();      // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks();  // Check if any callbacks are waiting to be processed.
}

inline void __attribute__ ((always_inline))  readVariousSensors(){
  /* Math functions that need to b calculated for each sensor are the following :
   * brakePressure = (ReadVoltage(A2)-0.5)*11.5;                           
   * tps = ReadVoltage(A1)/SupplyVoltage;                                  
   * potValue	= ((analogRead(potPin)/1023.0)-0.5)*SteeringWheel_RotationalRange/2;  
  */

  packt.CounterPulses = TCNT5;                                                // Get the counter pulses

  // analogReference(INTERNAL1V1);				                                       // Choose 1.1V internal reference
  ADMUX = (ADMUX & 0x3F) | (1 << REFS0) | (1 << REFS1);                       // Set Vref to internal 1V1
  ADMUX = (ADMUX & 0xFC) | ADC_MUX_SELECT;                                    // Set the MUX to the first ADC pin

  // Start the conversion
  ADCSRA |= (1 << ADSC);  
}



void setup() {
  Serial.begin(BAUD_RATE);
  Serial.println("\n\t!!-Initializing the DAQ System-!!");

  exportFunc = VoidExportFunc;

  /* Pin declerations */
  Serial.print("Initializing pins ");
  pinMode(PIN_START_BUTTON, INPUT_PULLUP);
  pinMode(PIN_STOP_BUTTON, INPUT_PULLUP);
  pinMode(PIN_RESET_BUTTON, INPUT_PULLUP);
  pinMode(CLK_PIN, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED,HIGH);
  delay(500);
  digitalWrite(LED,LOW);
  Serial.println("\t\t\tPASS");

  Serial.print("Initializing pins ");
  ADC_init();
  Serial.println("\t\t\tPASS");

  Serial.print("Initializing Watchdog :");
  wdt_enable(WDTO_8S);                                                        // Enable watchdog with a 8-second timeout
  Serial.println("\t\t\tPASS");

  Serial.println("Initializing IMU :");
  mpu.init();                                                             // Initialize the MPU6050

  wdt_reset();                                                                // Reset the watchdog timer

  Serial.print("\nInitializing SD card :");
  // if (!SD.begin(53)) {                                           // SD card initialization
    // ERROR = true;
    // ErrorFlag = 0x05;
  //   Serial.println("SD initialization failed!\n\tFix and reboot to contrinue\n");
  //   return;
  // }

  wdt_reset();                                                                // Reset the watchdog timer

  // dataFile = SD.open("data.txt", FILE_WRITE);                                 // Open the data file
  // dataFile ? Serial.println("\t\t\tPASS") : Serial.println("\t\t\t|--FAIL--|");

  wdt_reset();                                                                // Reset the watchdog timer

  Serial.print("Initializing GPS :");
  Serial.flush();
  initGPS(&myGNSS);
  Serial.println("\t\t\tPASS");

  wdt_reset();                                                                // Reset the watchdog timer

  Serial.print("Setting up the interrupts :");
  attachInterrupt(digitalPinToInterrupt(PIN_START_BUTTON), ISR_startRecording, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_STOP_BUTTON), ISR_stopRecording, FALLING);
  Serial.println("\t\tPASS ");

  wdt_reset();                                                                // Reset the watchdog timer

  Serial.print("Initializing Timers :");
  if (initTimers(__SAMPLING_RATE__)){
    Serial.println("\t\t\tPASS");
  } else {
    // ERROR = true;
    // ErrorFlag = 0x01;
    Serial.println("\t\t|--FAIL--|");
  }

  wdt_reset();                                                                // Reset the watchdog timer

  Serial.println("\n\t!!-Initialization complete-!!");
  // Serial.flush();
}

void loop()	{
  wdt_reset();                                                                // Reset the watchdog timer

  if(Serial.available()){
    char buffer[32] = "\0";
    int len = Serial.readBytesUntil('\n',buffer, 32);

    command_t command = SerialCommander.handleCommand(buffer, len);

    if (command.function != NULL){
      Serial.println(command.message);
      command.function(command.argc, command.argv);
    } else {
      Serial.println("invalid command");
    }
  }
  
  if	(recording)	{
    if (!SAMPLE_WINDOW) return;

    digitalWrite(46,HIGH);
    readGPS();			                                                          // Wait	for	VTG	messsage and print speed in	Serial.	 

    /*  Read data from	MPU6050 */
    mpu.get6AxMotion(&packt.accelerationX, &packt.accelerationY, &packt.accelerationZ, &packt.gyroX, &packt.gyroY, &packt.gyroZ);

    readVariousSensors();

    packt.elapsedTime = micros() - StartTime;                                   // Calculate the elapsed time   

    exportFunc();

    SAMPLE_WINDOW = false;                                                      // Close the "Window"

    digitalWrite(46,LOW);
  }
}