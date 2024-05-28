#include <MPU6050_6Axis_MotionApps612.h>
#include "../general/general.h"

class IMU{
    private:
    MPU6050 mpu;

    public:
        bool init();
        bool get6AxMotion(int *accX, int *accY, int *accZ, int* gyroX, int* gyroY, int* gyroZ);
};