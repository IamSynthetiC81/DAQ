#include <MPU6050_6Axis_MotionApps612.h>
#include "../general/general.h"

class IMU{
	public:
    bool init(uint8_t deviceAddress);
    bool get6AxMotion(int *accX, int *accY, int *accZ, int* gyroX, int* gyroY, int* gyroZ);
	private:
    MPU6050 mpu;

		uint8_t deviceAddress;

		uint8_t findDevices(uint8_t* add);
};