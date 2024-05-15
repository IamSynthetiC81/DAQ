static volatile uint8_t VEC[8];

#define ERREG_GPS 0
#define ERREG_IMU 1
#define ERREG_SDC 2
#define ERREG_ARD 3
#define ERREG_INT 4
#define ERREG_TIM 5
#define ERREG_GN1 6
#define ERREG_GN2 7

enum GPS_ERROR_CODES{
    BAUD_RATE_MISMATCH = 0,
    PORT_INACTIVE = 1,
};

enum IMU_ERROR_CODES{
    ADDRESS_NOT_FOUND = 0,
};

enum SDC_ERROR_CODES{
    FILE_NOT_FOUND = 0,
    SD_NOT_PRESENT = 1,
};

enum ARD_ERROR_CODES{
    SAMPLING_RATE_HIGH = 0,
    SAMPLING_RATE_LOW = 1,
};

enum INT_ERROR_CODES{
    INTERRUPT_NOT_FOUND = 0,
    INTERRUPT_CORRUPT = 1,
    INTERRUPT_MISSING = 2,
    INTERRUPT_OUT_OF_BOUNDS = 3,
};

enum TIM_ERROR_CODES{
    TIMER_UNAVAILABLE = 0,
    TIMER_OVERFLOW = 1,
};

enum GN1_ERROR_CODES{
    PROTOCOL_I2C_STUCK_BUS = 0,
    PROTOCOL_SERIAL_UNAVAILABLE = 1,
};

void clearError(){
  int length = sizeof(VEC);

  for (int i = 0 ; i < length ; i++ ){
    VEC[i] = 0x00;
  }
}