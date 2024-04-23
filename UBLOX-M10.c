// // #include <avr/pgmspace.h>


// #ifndef UBLOX_M10_H
//     #define UBLOX_M10_H
// #endif

// // Function to send a UBX message via Serial
// void sendUBX(uint8_t *msg, uint8_t len) {
//   for (uint8_t i = 0; i < len; i++) {
//     Serial.write(msg[i]);
//   }
// }


// bool PollUBX_MON_VER() {
//   uint8_t UBX_MON_VER[] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x31};
//   sendUBX(UBX_MON_VER, sizeof(UBX_MON_VER) / sizeof(UBX_MON_VER[0]));
//   return true;
// }

//   void UBX_HIGH_SPEED_CPU_CLOCK(){
//     uint8_t configMsg[] = {0xB5, 0x62, 0x06, 0x8A, 0x16, 0x00, 0x10, 0x0E, 0x03, 0x00, 0x00, 0x01, 0x00, 0xA4, 0x40, 0x03, 0x00, 0xA4, 0x40, 0x05, 0x00, 0xA4, 0x40, 0x0A, 0x00, 0xA4, 0x40, 0x4C, 0x15, 0xF5, 0xB8};
//     sendUBX(configMsg, sizeof(configMsg) / sizeof(configMsg[0]));
//   }



// const uint8_t PROGMEM HighCPU_Clock[] = "B5 62 06 41 10 00 03 00 04 1F 54 5E 79 BF 28 EF 12 05 FD FF FF FF 8F 0D B5 62 06 41 1C 00 04 01 A4 10 BD 34 F9 12 28 EF 12 05 05 00 A4 40 00 B0 71 0B 0A 00 A4 40 00 D8 B8 05 DE AE";

