#include <stdint.h>
#include "Stream.h"
#include "HardwareSerial.h"

#define GPS_DEFAULT_BAUDRATE 9600

unsigned long baudrate;
unsigned short rate;

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


bool GPS_init(Stream logStream, Stream gpsStream, uint16_t targetBaudrate,  uint16_t targetRate){
  
  baudrate = targetBaudrate;
  rate = targetRate; 
  
  // Restore the default GPS receiver configuration.
  for (byte i = 0; i < sizeof(gpsPossibleBaudrates) / sizeof(*gpsPossibleBaudrates); i++){
    
    logSerial.print("Trying to restore defaults at ");
    logSerial.print(gpsPossibleBaudrates[i]);
    logSerial.println(" baudrate...");

    if (i != 0){
        delay(100); // Little delay before the flush.
        gpsStream.flush();
    }

    gpsStream.begin(gpsPossibleBaudrates[i]);
    restoreDefaults();
  }

  // Switch the GPS receiver serial configuration to the default baudrate.
  if (gpsPossibleBaudrates[sizeof(gpsPossibleBaudrates) / sizeof(*gpsPossibleBaudrates) - 1] != GPS_DEFAULT_BAUDRATE){
    
    logSerial.print("Switching to the default baudrate which is ");
    logSerial.print(GPS_DEFAULT_BAUDRATE);
    logSerial.println("...");

    delay(100); // Little delay before the flush.
    gpsSerial.flush();
    gpsSerial.begin(GPS_DEFAULT_BAUDRATE);
  }

  // Switch the GPS receiver serial configuration to the target baudrate.
  if (baudrate != GPS_DEFAULT_BAUDRATE){
      logSerial.print("Switching to the target baudrate which is ");
      logSerial.print(baudrate);
      logSerial.println("...");

      changeBaudrate();

      delay(100); // Little delay before the flush.
      gpsSerial.flush();
      gpsSerial.begin(baudrate);
  }
  
  if (rate == 100){
    // Change receiving frequency to 100 ms.
    logSerial.println("Changing receiving frequency to 100 ms...");
    changeFrequency();
  }

  // Disable unnecessary channels like SBAS or QZSS.
  // logSerial.println("Disabling unnecessary channels...");
  // disableUnnecessaryChannels();

  // Enable NAV-PVT messages.
  // logSerial.println("Enabling NAV-PVT messages...");
  // enableNavPvt();

  logSerial.println("Auto-configuration is complete!");

  delay(100); // Little delay before the flush.
  gpsSerial.flush();

}


void sendPacket(byte *packet, byte len){
    for (byte i = 0; i < len; i++)
    {
        gpsSerial.write(packet[i]);
    }

    printPacket(packet, len);
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