#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SD.h>
#include "NMEA_Parser.h"
#include <math.h>

static uint8_t SupplyVoltage = 5;
#define SupplyVoltageSense_Ratio 5.99
#define SupplyVoltageSense_Pin 0

#define ADC_bit_range 1023                                                // 10 bit ADC;
#define ReadVoltage(Pin) (SupplyVoltage*analogRead(Pin)/ADC_bit_range)    // Reads Voltage

#define SteeringWheel_RotationalRange 270

typedef struct information{
  uint8_t counter;
  float* accelerationX;
  float* accelerationY;
  float* accelerationZ;
  float* gyroX;
  float* gyroY;
  float* gyroZ;
  float roll;
  float pitch;
  float yaw;
  float brakePressure;
  float tps;
  float samplingRate;
  unsigned long elapsedTime;
  float elapsedTimeSeconds;
  byte potValue;
  float SpeedGPS;
} log_t;


void CalibrateSupplyVoltage(int Pin){
  analogReference(INTERNAL1V1);             // Choose 1.1V internal reference
  SupplyVoltage = SupplyVoltageSense_Ratio*ReadVoltage(SupplyVoltageSense_Pin);
  analogReference(DEFAULT);                 // Choose the default reference
}

Adafruit_MPU6050 mpu;
File dataFile;

static const int startButtonPin = 2;
const int stopButtonPin = 3;
const int resetButtonPin = 9;
const int ledPin = 5;

bool recording = false;
int counter = 0;
unsigned long previousMillis = 0;
unsigned long interval = 1;

// float gyroX, gyroY, gyroZ;
// float yaw = 0.0; // Initialize yaw angle

// Define the analog pin where the potentiometer is connected
const int potPin = A1;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(10);
  }

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  pinMode(startButtonPin, INPUT_PULLUP);
  pinMode(stopButtonPin, INPUT_PULLUP);
  pinMode(resetButtonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  // Initialize SD card
  if (!SD.begin()) {
    Serial.println("SD initialization failed!");
    while (1);
  }

  dataFile = SD.open("data.txt", FILE_WRITE);

  if (dataFile) {
    dataFile.println("Counter\tAccX\tAccY\tAccZ\tRoll\tPitch\tYaw\tGyroX\tGyroY\tGyroZ\tBrakePressure\tTPS\tSamplingRate\tElapsedTime\tPotentiometerValue");
    dataFile.flush();
  } else {
    Serial.println("Error opening data.txt");
  }

  Serial1.begin(9600);        // Serial1 port connected to GPS
}

void SerialPrintLog(log_t log){
      // Print data to Serial
      Serial.print("Counter: ");
      Serial.print(log.counter);
      Serial.print("\t");

      Serial.print("AccX: ");
      Serial.print(*log.accelerationX);
      Serial.print("\t");

      Serial.print("AccY: ");
      Serial.print(*log.accelerationY);
      Serial.print("\t");

      Serial.print("AccZ: ");
      Serial.print(*log.accelerationZ);
      Serial.print("\t");

      Serial.print("Roll: ");
      Serial.print(log.roll);
      Serial.print("\t");

      Serial.print("Pitch: ");
      Serial.print(log.pitch);
      Serial.print("\t");

      Serial.print("Yaw: ");
      Serial.print(log.yaw);
      Serial.print("\t");

      Serial.print("GyroX: ");
      Serial.print(*log.gyroX);
      Serial.print("\t");

      Serial.print("GyroY: ");
      Serial.print(*log.gyroY);
      Serial.print("\t");

      Serial.print("GyroZ: ");
      Serial.print(*log.gyroZ);
      Serial.print("\t");

      Serial.print("BrakePressure: ");
      Serial.print(log.brakePressure);
      Serial.print("\t");

      Serial.print("TPS: ");
      Serial.print(log.tps);
      Serial.print("\t");

      Serial.print("Sampling Rate: ");
      Serial.print(log.samplingRate);
      Serial.print(" Hz\t");

      Serial.print("ElapsedTime: ");
      Serial.print(log.elapsedTime);
      Serial.print(" sec\t");

      Serial.print("PotentiometerValue: ");
      Serial.println(log.potValue);

}

void readGPS(log_t* log){
  if(Serial.available()){
    int bufsize = 256;
    char buf[bufsize];
    String something = Serial.readStringUntil('\n');      // Read the GPS data
    
    something.toCharArray(buf,bufsize);                   // Convert the GPS data to char array
    
    packet_t packet = read(buf);
    
    if (packet.format == VTG)
      log->SpeedGPS = packet.message.VTG.SpeedOverGround;
  }
}
void readMPU6050(Adafruit_MPU6050 mpu ,log_t* log){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  log->accelerationX = &a.acceleration.x;
  log->accelerationY = &a.acceleration.y;
  log->accelerationZ = &a.acceleration.z;
  log->gyroX = &g.gyro.x;
  log->gyroY = &g.gyro.y;
  log->gyroZ = &g.gyro.z;
}
void readVariousSensors(log_t* log){
  log->brakePressure = (analogRead(A2)-0.5);
  log->tps = ReadVoltage(A1)/SupplyVoltage;
  log->potValue = ((analogRead(potPin)/1023.0)-0.5)*SteeringWheel_RotationalRange/2;
}
void Calc_Rotation_XYZ(log_t* log){
  log->roll  = atan2(*log->accelerationY, *log->accelerationZ) * 180.0 / PI;
  log->pitch = atan2(-(*log->accelerationX), sqrt(*log->accelerationY * (*log->accelerationY) + pow((*log->accelerationZ),2))) * 180.0 / PI;
  log->yaw += (*log->gyroZ / 131.0) * (interval / 1000.0); // 131.0 is the sensitivity scale factor for the gyro (datasheet)
}
void WriteToSD(const log_t* log){
  // Write data to SD card
  dataFile.print(log->counter);
  dataFile.print("\t");

  dataFile.print(*log->accelerationX);
  dataFile.print("\t");
  dataFile.print(*log->accelerationY);
  dataFile.print("\t");
  dataFile.print(*log->accelerationZ);
  dataFile.print("\t");
  dataFile.print(log->roll);
  dataFile.print("\t");
  dataFile.print(log->pitch);
  dataFile.print("\t");
  dataFile.print(log->yaw);
  dataFile.print("\t");
  dataFile.print(*log->gyroX);
  dataFile.print("\t");
  dataFile.print(*log->gyroY);
  dataFile.print("\t");
  dataFile.print(*log->gyroZ);
  dataFile.print("\t");
  dataFile.print(log->brakePressure);
  dataFile.print("\t");
  dataFile.print(log->tps);
  dataFile.print("\t");
  dataFile.print(log->elapsedTime);
  dataFile.print("\t");
  dataFile.print(log->potValue);
  dataFile.print(log->SpeedGPS);

  dataFile.flush();
}

void loop() {
  unsigned long currentMillis = millis();

  static log_t *log;

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (digitalRead(startButtonPin) == LOW && !recording) {
      startRecording();
    }

    if (digitalRead(stopButtonPin) == LOW && recording) {
      stopRecording();
    }

    if (digitalRead(resetButtonPin) == LOW) {
      resetCounter();
    }

    if (recording) {
      digitalWrite(ledPin, HIGH);

      // float accelerationX = a.acceleration.x;
      // float accelerationY = a.acceleration.y;
      // float accelerationZ = a.acceleration.z;
      // float roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
      // float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

      // gyroX = g.gyro.x;
      // gyroY = g.gyro.y;
      // gyroZ = g.gyro.z;

    
      // Calculate yaw using gyro readings
      // yaw += (gyroZ / 131.0) * (interval / 1000.0); // 131.0 is the sensitivity scale factor for the gyro (datasheet)

      // float brakePressure = analogRead(A2) * (4.5 / 1023) * 10.21; // Convert analog reading to brake pressure (in psi)
      // float tps = (analogRead(A1) / 1023.0) * 100; // Calculate TPS percentage

      

      // float samplingRate = 1000.0 / interval;
      float elapsedTimeSec = currentMillis / 1000.0;

      readMPU6050(mpu,log);
      readGPS(log);                                            // Wait for VTG messsage and print speed in serial.  
      readVariousSensors(log);

      WriteToSD(log);
    } else {
      digitalWrite(ledPin, LOW);
    }
  }
}

void startRecording() {
  recording = true;
  Serial.println("Recording started");
}

void stopRecording() {
  recording = false;
  dataFile.close();
  Serial.println("Recording stopped");
}

void resetCounter() {
  static unsigned long lastResetTime = 0;
  static bool resetButtonPressed = false;

  if (digitalRead(resetButtonPin) == LOW) {
    if (!resetButtonPressed) {
      unsigned long currentTime = millis();
      if (currentTime - lastResetTime > 500) {
        counter++;
        if (counter >= 3) {
          counter = 0;
          Serial.println("Counter auto-reset");
        } else {
          Serial.print("Counter reset to: ");
          Serial.println(counter);
        }
        lastResetTime = currentTime;
      }
      resetButtonPressed = true;
    }
  } else {
    resetButtonPressed = false;
  }
}
