#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SD.h>

#include <math.h>

static uint8_t SupplyVoltage = 5;
#define SupplyVoltageSense_Ratio 5.99;
#define SupplyVoltageSense_Pin
#define SupplyVoltage_Calc analogReference(INTERNAL);SupplyVoltage = SupplyVoltageSense_Ratio*analogRead(Pin);analogReference(DEFAULT);


#define adc_bit_width 10;
#define ReadVoltage(Pin) SupplyVoltage*AnalogRead(Pin)/pow(2,adc_bit_width); 

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
  int potValue;
  float SpeedGPS;
} log_t;

void CalibrateSupplyVoltage(uint8_t Pin){
  analogReference(INTERNAL);                // Choose 1.1V internal reference
  SupplyVoltage = SupplyVoltageSense_Ratio*analogRead(Pin);
  analogReference(DEFAULT);                 // Choose the default reference
}

Adafruit_MPU6050 mpu;
File dataFile;

const int startButtonPin = 2;
const int stopButtonPin = 3;
const int resetButtonPin = 9;
const int ledPin = 5;

bool recording = false;
int counter = 0;
unsigned long previousMillis = 0;
unsigned long interval = 1;

float gyroX, gyroY, gyroZ;
float yaw = 0.0; // Initialize yaw angle

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

void readGPS(){
  if(Serial.available()){
    String something = Serial.readStringUntil('\n');      // Read the GPS data
    // Serial.print(something);                           // Print the GPS data
    something.toCharArray(buf,bufsize);                   // Convert the GPS data to char array
    // Serial.print(buf);                                 // Print the char array           
    packet_t packet = read(buf);
    
    if (packet.format == VTG)
      // VTG_t message = packet.message.VTG;
      Serial.print(packet.message.VTG.message.SpeedOverGround)
      Serial.println("Km/h");
  }
}

void loop() {
  unsigned long currentMillis = millis();

  const log_t log;

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

      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      log.accelerationX = &a.acceleration.x;
      log.accelerationY = &a.acceleration.y;
      log.accelerationZ = &a.acceleration.z;
      log.roll = roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
      log.pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

      // float accelerationX = a.acceleration.x;
      // float accelerationY = a.acceleration.y;
      // float accelerationZ = a.acceleration.z;
      // float roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
      // float pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

      // gyroX = g.gyro.x;
      // gyroY = g.gyro.y;
      // gyroZ = g.gyro.z;

      log.gyroX = g.gyro.x;
      log.gyroY = g.gyro.y;
      log.gyroZ = g.gyro.z;

      // Calculate yaw using gyro readings
      // yaw += (gyroZ / 131.0) * (interval / 1000.0); // 131.0 is the sensitivity scale factor for the gyro (datasheet)
      log.yaw += (gyroZ / 131.0) * (interval / 1000.0); // 131.0 is the sensitivity scale factor for the gyro (datasheet)

      float brakePressure = analogRead(A2) * (4.5 / 1023) * 10.21; // Convert analog reading to brake pressure (in psi)
      float tps = (analogRead(A1) / 1023.0) * 100; // Calculate TPS percentage

      log.brakePressure = (analogRead(A2)-0.5)

      float samplingRate = 1000.0 / interval;
      unsigned long elapsedTime = currentMillis;
      float elapsedTimeSec = elapsedTime / 1000.0;

      // Read potentiometer value
      int potValue = analogRead(potPin);


      readGPS();                                            // Wait for VTG messsage and print speed in serial.  


      // Print data to Serial
      Serial.print("Counter: ");
      Serial.print(counter);
      Serial.print("\t");

      Serial.print("AccX: ");
      Serial.print(accelerationX);
      Serial.print("\t");

      Serial.print("AccY: ");
      Serial.print(accelerationY);
      Serial.print("\t");

      Serial.print("AccZ: ");
      Serial.print(accelerationZ);
      Serial.print("\t");

      Serial.print("Roll: ");
      Serial.print(roll);
      Serial.print("\t");

      Serial.print("Pitch: ");
      Serial.print(pitch);
      Serial.print("\t");

      Serial.print("Yaw: ");
      Serial.print(yaw);
      Serial.print("\t");

      Serial.print("GyroX: ");
      Serial.print(gyroX);
      Serial.print("\t");

      Serial.print("GyroY: ");
      Serial.print(gyroY);
      Serial.print("\t");

      Serial.print("GyroZ: ");
      Serial.print(gyroZ);
      Serial.print("\t");

      Serial.print("BrakePressure: ");
      Serial.print(brakePressure);
      Serial.print("\t");

      Serial.print("TPS: ");
      Serial.print(tps);
      Serial.print("\t");

      Serial.print("Sampling Rate: ");
      Serial.print(samplingRate);
      Serial.print(" Hz\t");

      Serial.print("ElapsedTime: ");
      Serial.print(elapsedTimeSec);
      Serial.print(" sec\t");

      Serial.print("PotentiometerValue: ");
      Serial.println(potValue);

      // Write data to SD card
      dataFile.print(counter);
      dataFile.print("\t");

      dataFile.print(accelerationX);
      dataFile.print("\t");
      dataFile.print(accelerationY);
      dataFile.print("\t");
      dataFile.print(accelerationZ);
      dataFile.print("\t");
      dataFile.print(roll);
      dataFile.print("\t");
      dataFile.print(pitch);
      dataFile.print("\t");
      dataFile.print(yaw);
      dataFile.print("\t");
      dataFile.print(gyroX);
      dataFile.print("\t");
      dataFile.print(gyroY);
      dataFile.print("\t");
      dataFile.print(gyroZ);
      dataFile.print("\t");
      dataFile.print(brakePressure);
      dataFile.print("\t");
      dataFile.print(tps);
      dataFile.print("\t");
      dataFile.print(elapsedTimeSec);
      dataFile.print("\t");
      dataFile.println(potValue);

      dataFile.flush();
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
