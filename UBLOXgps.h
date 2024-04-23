#include <stdint.h>
#include "Stream.h"
#include "HardwareSerial.h"

#define GPS_DEFAULT_BAUDRATE 9600

unsigned long baudrate;
unsigned short rate;

HardwareSerial *__LOG_STREAM__ = NULL, *__GPS_STREAM__ = NULL;

/* An array of possible baudrates that can be used by the GPS receiver, sorted descending to prevent excess Serial
 * flush/begin after restoring defaults. Uncomment values that can be used by the GPS receiver before the
 * auto-configuration.
 */ 
const long gpsPossibleBaudrates[] = {
    // 921600,
    // 460800,
    // 230400,
    115200,
    // 57600,
    38400,
    // 19200,
    9600,
    // 4800,
};

// Print a packet to the log serial port in a hexadecimal form.
uint8_t printPacket(byte *packet, byte len){
  if (__LOG_STREAM__) return 0xff;

  char temp[3];

  for (byte i = 0; i < len; i++){
    sprintf(temp, "%.2X", packet[i]);
    __LOG_STREAM__->print(temp);

    if (i != len - 1)
        __LOG_STREAM__->print(' ');
  }

  __LOG_STREAM__->println();

  return 0x01;
}

void sendPacket(byte *packet, byte len){
  if (__LOG_STREAM__) return 0xff;
  if (__GPS_STREAM__) return 0xff;

    for (byte i = 0; i < len; i++){
        __GPS_STREAM__->write(packet[i]);
    }

    printPacket(packet, len);
}

void restoreDefaults(){
  // CFG-CFG packet.
  byte packet[] = {
      0xB5, // sync char 1
      0x62, // sync char 2
      0x06, // class
      0x09, // id
      0x0D, // length
      0x00, // length
      0xFF, // payload
      0xFF, // payload
      0x00, // payload
      0x00, // payload
      0x00, // payload
      0x00, // payload
      0x00, // payload
      0x00, // payload
      0xFF, // payload
      0xFF, // payload
      0x00, // payload
      0x00, // payload
      0x17, // payload
      0x2F, // CK_A
      0xAE, // CK_B
  };

  sendPacket(packet, sizeof(packet));
}

void changeBaudrate(){
    // CFG-PRT packet.
    byte packet[] = {
        0xB5, // sync char 1
        0x62, // sync char 2
        0x06, // class
        0x00, // id
        0x14, // length
        0x00, // length
        0x01, // payload
        0x00, // payload
        0x00, // payload
        0x00, // payload
        0xD0, // payload
        0x08, // payload
        0x00, // payload
        0x00, // payload
        0x00, // payload
        0xC2, // payload
        0x01, // payload
        0x00, // payload
        0x07, // payload
        0x00, // payload
        0x03, // payload
        0x00, // payload
        0x00, // payload
        0x00, // payload
        0x00, // payload
        0x00, // payload
        0xC0, // CK_A
        0x7E, // CK_B
    };

    sendPacket(packet, sizeof(packet));
}

// Send a packet to the GPS receiver to change the frequency to 100 ms.
void changeFrequency(){
  // CFG-RATE packet.
  byte packet[] = {
    0xB5, // sync char 1
    0x62, // sync char 2
    0x06, // class
    0x08, // id
    0x06, // length
    0x00, // length
    0x64, // payload
    0x00, // payload
    0x01, // payload
    0x00, // payload
    0x01, // payload
    0x00, // payload
    0x7A, // CK_A
    0x12, // CK_B
  };
  sendPacket(packet, sizeof(packet));
}

bool GPS_init(HardwareSerial *logStream, HardwareSerial *gpsStream, long targetBaudrate,  long targetRate){
  
  baudrate = targetBaudrate;
  rate = targetRate; 

  __LOG_STREAM__ = logStream;
  __GPS_STREAM__ = gpsStream;
  
  // Restore the default GPS receiver configuration.
  for (byte i = 0; i < sizeof(gpsPossibleBaudrates) / sizeof(*gpsPossibleBaudrates); i++){
    
    __LOG_STREAM__->print("Trying to restore defaults at ");
    __LOG_STREAM__->print(gpsPossibleBaudrates[i]);
    __LOG_STREAM__->println(" baudrate...");

    if (i != 0){
        delay(100); // Little delay before the flush.
        __GPS_STREAM__->flush();
    }

    __GPS_STREAM__->begin(gpsPossibleBaudrates[i]);
    restoreDefaults();
  }

  // Switch the GPS receiver serial configuration to the default baudrate.
  if (gpsPossibleBaudrates[sizeof(gpsPossibleBaudrates) / sizeof(*gpsPossibleBaudrates) - 1] != GPS_DEFAULT_BAUDRATE){
    
    __LOG_STREAM__->print("Switching to the default baudrate which is ");
    __LOG_STREAM__->print(GPS_DEFAULT_BAUDRATE);
    __LOG_STREAM__->println("...");

    delay(100); // Little delay before the flush.
    __GPS_STREAM__->flush();
    __GPS_STREAM__->begin(GPS_DEFAULT_BAUDRATE);
  }

  // Switch the GPS receiver serial configuration to the target baudrate.
  if (baudrate != GPS_DEFAULT_BAUDRATE){
      __LOG_STREAM__->print("Switching to the target baudrate which is ");
      __LOG_STREAM__->print(baudrate);
      __LOG_STREAM__->println("...");

      changeBaudrate();

      delay(100); // Little delay before the flush.
      __GPS_STREAM__->flush();
      __GPS_STREAM__->begin(baudrate);
  }
  
  if (rate == 100){
    // Change receiving frequency to 100 ms.
    __LOG_STREAM__->println("Changing receiving frequency to 100 ms...");
    changeFrequency();
  }

  // Disable unnecessary channels like SBAS or QZSS.
  // logSerial.println("Disabling unnecessary channels...");
  // disableUnnecessaryChannels();

  // Enable NAV-PVT messages.
  // logSerial.println("Enabling NAV-PVT messages...");
  // enableNavPvt();

  __LOG_STREAM__->println("Auto-configuration is complete!");

  delay(100); // Little delay before the flush.
  __GPS_STREAM__->flush();

}