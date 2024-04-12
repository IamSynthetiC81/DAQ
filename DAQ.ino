#include <FreqMeasure.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SD.h>
#include "NMEA_Parser.h"
#include <math.h>

static uint8_t SupplyVoltage = 5;
#define	SupplyVoltageSense_Ratio 5.99
#define	SupplyVoltageSense_Pin 0

#define	ADC_bit_range 1023												                          // 10 bit ADC;
#define	ReadVoltage(Pin) (SupplyVoltage*analogRead(Pin)/ADC_bit_range)	    //	Reads Voltage

#define	SteeringWheel_RotationalRange 270

typedef	struct information{
  uint8_t counter;
  float accelerationX;
  float accelerationY;
  float accelerationZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float	roll;
  float	pitch;
  float	yaw;
  float	brakePressure;
  float	tps;
  float	samplingRate;
  unsigned long	elapsedTime;
  float	elapsedTimeSeconds;
  byte potValue;
  float	SpeedGPS;
} log_t;

log_t __LOG;

void CalibrateSupplyVoltage(int	Pin){
  analogReference(INTERNAL1V1);				// Choose 1.1V internal reference
  SupplyVoltage	= SupplyVoltageSense_Ratio*ReadVoltage(SupplyVoltageSense_Pin);
  analogReference(DEFAULT);					// Choose the default reference
}

Adafruit_MPU6050 mpu;
File dataFile;

static const int startButtonPin	= 2;
const int stopButtonPin	= 3;
const int resetButtonPin = 9;
const int ledPin = LED_BUILTIN;

bool recording = true;
int	counter	= 0;
unsigned long previousMillis = 0;
unsigned long interval = 1;

// float gyroX,	gyroY, gyroZ;
// float yaw = 0.0;	// Initialize yaw angle

// Define the analog pin where the potentiometer is	connected
const int potPin = A1;

void setup() {
  Serial.begin(9600);
  // while	(!Serial) {
	//   delay(10);
  // }

  if (!mpu.begin())	{
    Serial.println("Failed	to find	MPU6050	chip\n\tFix and reboot to continue\n");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  pinMode(startButtonPin, INPUT_PULLUP);
  pinMode(stopButtonPin, INPUT_PULLUP);
  pinMode(resetButtonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  // Initialize	SD card
  if (!SD.begin(10)) {
    Serial.println("SD	initialization failed!\n\tFix and reboot to contrinue\n");
    while (1);
  }

  dataFile = SD.open("data.txt", FILE_WRITE);

  if (dataFile)	{
    dataFile.println("Counter\tAccX\tAccY\tAccZ\tRoll\tPitch\tYaw\tGyroX\tGyroY\tGyroZ\tBrakePressure\tTPS\tSamplingRate\tElapsedTime\tPotentiometerValue");
    dataFile.flush();
  }	else {
  	Serial.println("Error opening data.txt");
  }

  Serial1.begin(9600);		  // Serial1 port connected	to GPS
  FreqMeasure.begin();
}

void SerialPrintLog(){
	  // Print	data to	Serial
	  Serial.print("Counter: ");
	  Serial.print(__LOG.counter);
	  Serial.print("\t");

	  Serial.print("AccX: ");
	  Serial.print(__LOG.accelerationX);
	  Serial.print("\t");

	  Serial.print("AccY: ");
	  Serial.print(__LOG.accelerationY);
	  Serial.print("\t");

	  Serial.print("AccZ: ");
	  Serial.print(__LOG.accelerationZ);
	  Serial.print("\t");

	  Serial.print("Roll: ");
	  Serial.print(__LOG.roll);
	  Serial.print("\t");

	  Serial.print("Pitch:	");
	  Serial.print(__LOG.pitch);
	  Serial.print("\t");

	  Serial.print("Yaw: ");
	  Serial.print(__LOG.yaw);
	  Serial.print("\t");

	  Serial.print("GyroX:	");
	  Serial.print(__LOG.gyroX);
	  Serial.print("\t");

	  Serial.print("GyroY:	");
	  Serial.print(__LOG.gyroY);
	  Serial.print("\t");

	  Serial.print("GyroZ:	");
	  Serial.print(__LOG.gyroZ);
	  Serial.print("\t");

	  Serial.print("BrakePressure:	");
	  Serial.print(__LOG.brakePressure);
	  Serial.print("\t");

	  Serial.print("TPS: ");
	  Serial.print(__LOG.tps);
	  Serial.print("\t");

	  Serial.print("Sampling Rate:	");
	  Serial.print(__LOG.samplingRate);
	  Serial.print(" Hz\t");

	  Serial.print("ElapsedTime: ");
	  Serial.print(__LOG.elapsedTime);
	  Serial.print(" sec\t");

	  Serial.print("PotentiometerValue: ");
	  Serial.println(__LOG.potValue);

}


void readGPS(){

  static uint8_t speed = 0;

  if(Serial1.available()){
    int bufsize = 256;
    char buf[bufsize];
    String	something =	Serial.readStringUntil('\n');	   //	Read the GPS data
    
    something.toCharArray(buf,bufsize);				   // Convert the GPS data to char array
    
    packet_t packet = read(buf);
    if	(packet.format == VTG)
    __LOG.SpeedGPS = packet.message.VTG.SpeedOverGround;
  }
}
void readMPU6050(Adafruit_MPU6050 mpu ){
  sensors_event_t a, g,	temp;
  mpu.getEvent(&a, &g, &temp);

  __LOG.accelerationX = a.acceleration.x;
  __LOG.accelerationY = a.acceleration.y;
  __LOG.accelerationZ = a.acceleration.z;
  __LOG.gyroX = g.gyro.x;
  __LOG.gyroY = g.gyro.y;
  __LOG.gyroZ = g.gyro.z;
}
void readVariousSensors(){
  __LOG.brakePressure = (ReadVoltage(A2)-0.5)*11.5;
  __LOG.tps = ReadVoltage(A1)/SupplyVoltage;
  __LOG.potValue	= ((analogRead(potPin)/1023.0)-0.5)*SteeringWheel_RotationalRange/2;
}
void Calc_Rotation_XYZ(){
  __LOG.roll	 = atan2(__LOG.accelerationY, __LOG.accelerationZ) * 180.0 / PI;
  __LOG.pitch = atan2(-(__LOG.accelerationX), sqrt(__LOG.accelerationY *	(__LOG.accelerationY) +	pow((__LOG.accelerationZ),2))) * 180.0 / PI;
  __LOG.yaw += (__LOG.gyroZ / 131.0)	* (interval	/ 1000.0); // 131.0	is the sensitivity scale factor	for	the	gyro (datasheet)
}
void WriteToSD(){
  // Write data	to SD card
  dataFile.print(__LOG.counter);
  dataFile.print("\t");

  dataFile.print(__LOG.accelerationX);
  dataFile.print("\t");
  dataFile.print(__LOG.accelerationY);
  dataFile.print("\t");
  dataFile.print(__LOG.accelerationZ);
  dataFile.print("\t");
  dataFile.print(__LOG.roll);
  dataFile.print("\t");
  dataFile.print(__LOG.pitch);
  dataFile.print("\t");
  dataFile.print(__LOG.yaw);
  dataFile.print("\t");
  dataFile.print(__LOG.gyroX);
  dataFile.print("\t");
  dataFile.print(__LOG.gyroY);
  dataFile.print("\t");
  dataFile.print(__LOG.gyroZ);
  dataFile.print("\t");
  dataFile.print(__LOG.brakePressure);
  dataFile.print("\t");
  dataFile.print(__LOG.tps);
  dataFile.print("\t");
  dataFile.print(__LOG.elapsedTime);
  dataFile.print("\t");
  dataFile.print(__LOG.potValue);
  dataFile.print(__LOG.SpeedGPS);

  dataFile.flush();
}

void loop()	{
  unsigned long	currentMillis =	millis();
  Serial.println("Recording : ");
  
  // if (recording) Serial.println("ON");
  // else Serial.println("FALSE");

	// if	(digitalRead(startButtonPin) ==	LOW	&& !recording) {
	//   startRecording();
	// }

	// if	(digitalRead(stopButtonPin)	== LOW && recording) {
	//   stopRecording();
	// }

	// if	(digitalRead(resetButtonPin) ==	LOW) {
	//   resetCounter();
	// }

	if	(recording)	{

    double sum = 0;
    int count = 0;

    if(FreqMeasure.available()){
      sum =+FreqMeasure.read();
      count = count + 1;
      if (count > 30){
        float freq = FreqMeasure.countToFrequency(sum/count);
        sum = 0;
        count = 0;
      }
    }

    // readGPS();											  // Wait	for	VTG	messsage and print speed in	serial.	 

    if (currentMillis	- previousMillis >=	interval) {
	    previousMillis	= currentMillis;
	    digitalWrite(ledPin,	HIGH);
      Serial.println("Data");

	  // float	samplingRate = 1000.0 /	interval;
	  float elapsedTimeSec	= currentMillis	/ 1000.0;

	  // readMPU6050(mpu);									 // Read data from	MPU6050
	  
	  readVariousSensors();

	  WriteToSD();
    
    } else	{
      digitalWrite(ledPin,	LOW);
    }
  }
}

void startRecording() {
  recording	= true;
  Serial.println("Recording	started");
}

void stopRecording() {
  recording	= false;
  dataFile.close();
  Serial.println("Recording	stopped");
}

void resetCounter()	{
  static unsigned long lastResetTime = 0;
  static bool resetButtonPressed = false;

  if (digitalRead(resetButtonPin) == LOW) {
	if	(!resetButtonPressed) {
	  unsigned	long currentTime = millis();
	  if (currentTime - lastResetTime > 500) {
		counter++;
		if (counter >= 3)	{
		  counter	= 0;
		  Serial.println("Counter	auto-reset");
		}	else {
		  Serial.print("Counter reset	to:	");
		  Serial.println(counter);
		}
		lastResetTime	= currentTime;
	  }
	  resetButtonPressed =	true;
	}
  }	else {
	resetButtonPressed	= false;
  }
}
