#include "IMU.h"
#include <avr/wdt.h>

bool IMU::init(){
	uint8_t devStatus;                                                          // return status after each device operation (0 = PASS, !0 = error)
  uint16_t packetSize;                                                        // expected DMP packet size (default is 42 bytes)

  wdt_reset();

  log("\t|---Initializing I2C Drivers :");
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE                            // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000);                                                    // 400kHz I2C clock. Comment this line if code is not compiling
    log("\t\tPASS\n");
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    log("\t\tPASS\n");
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

  log("\t|---Initializing MPU6050:");
  mpu.initialize();
  log("\t\tPASS\n");

  log("\t\t|--Testing device connection:");                         // verify connection
  if (mpu.testConnection()){
    log("\t\tPASS"); 
    
    log("\t\t|--Initializing DMP");                                // load and configure the DMP
    // devStatus = mpu.dmpInitialize();
  } else
    log("\t\tFAILED <--");
  
  if (devStatus == 0) {                                                       // make sure it worked (returns 0 if so)
    wdt_reset();

    log("\t\t|--Calibrating:\n");                                    // Caibrate
		log("\t\t\t|--Accel :");
    mpu.CalibrateAccel(15);                                                   // Calibration Time: generate offsets and calibrate our MPU6050
		log("\tSUCCESS\n\t\t\t|--Gyro :");
    mpu.CalibrateGyro(15);
		log("\tSUCCESS\n");

    log("\n\t\t|--Enabling DMP:");
    // mpu.setDMPEnabled(true);
    log("\t\t\tPASS"); 

    packetSize = mpu.dmpGetFIFOPacketSize();                                  // get expected DMP packet size for later comparison

    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);                           // Set the accelerometer range to 4G
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    log("DMP Initialization failed (code ");
    log(devStatus);
    log(")");
  }
}

bool IMU::get6AxMotion(int *accX, int *accY, int *accZ, int* gyroX, int* gyroY, int* gyroZ){
	mpu.getMotion6(accX,accY,accZ,gyroX,gyroY,gyroZ);
}