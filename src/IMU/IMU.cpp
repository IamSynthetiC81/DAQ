#include "IMU.h"
#include <avr/wdt.h>

bool IMU::init(uint8_t deviceAddress){
  this->deviceAddress = deviceAddress;

	uint8_t devStatus;                                                          // return status after each device operation (0 = PASS, !0 = error)
  uint16_t packetSize;                                                        // expected DMP packet size (default is 42 bytes)

  wdt_reset();

  log("|---Initializing I2C Drivers :");
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE                            // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000);                                                    // 400kHz I2C clock. Comment this line if code is not compiling
    logln("\t\tPASS");
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    log("\t\tPASS\n");
  #endif

  // uint8_t *devicesAddress;
  // uint8_t devicesCount = findDevices(devicesAddress);                          // Find devices on the I2C bus
  // bool deviceFound = false;

  // if(devicesCount == 0){
  //   ERROR("No devices found on the I2C bus");
  //   return false;
  // } else {
  //   for(uint8_t i = 0; i < devicesCount; i++){
  //     if(devicesAddress[i] == deviceAddress){
  //       deviceFound = true;
  //       break;
  //     }
  //   }
  // }

  // if(!deviceFound ){
  //   if(devicesCount > 1){
  //     ERROR("Multiple devices found on the I2C bus but not the one specified");
  //   }else if (devicesCount == 1){
  //     ERROR("Device found but on a different address");
  //     Serial.print("Device found on address: ");
  //     Serial.println(devicesAddress[0], HEX);
  //   } else {
  //     ERROR("No devices found on the I2C bus");
  //   }
  //   return false;
  // }

  Wire.beginTransmission(0x68);                                               // Start I2C communication with the MPU6050

  log("\t|--Initializing MPU6050:");                                          // Initialize the MPU6050
  mpu.initialize();
  logln("\t\tPASS");

  log("\t|--Testing device connection:");                                     // verify connection
  if (mpu.testConnection()){
    logln("\t\tPASS"); 
    
    log("\t\t|--Initializing DMP");                                // load and configure the DMP
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0)
      logln("\t\t\tPASS");
  } else
    logln("\t\tFAILED <--");
  
  if (devStatus == 0) {                                                       // make sure it worked (returns 0 if so)
    wdt_reset();

    logln("\t\t|--Calibrating:");                                           // Caibrate
		log("\t\t\t|--Accel :");
    mpu.CalibrateAccel(15);                                                   // Calibration Time: generate offsets and calibrate our MPU6050
		// logln("\tSUCCESS");
    log("\n\t\t\t|--Gyro :");
    mpu.CalibrateGyro(15);
		// logln("\tSUCCESS");

    log("\n\t\t|--Enabling DMP:");
    mpu.setDMPEnabled(true);
    logln("\t\t\tPASS"); 

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

uint8_t IMU::findDevices(uint8_t* add){
  add = (uint8_t*)malloc(sizeof(uint8_t));

  uint8_t count = 0;

  for (uint8_t i = 0; i < 128; i++){
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0){
      add = (uint8_t*)realloc(add, (count + 1) * sizeof(uint8_t));
      add[count] = i;
      count++;
    }
  }
  return count;
}