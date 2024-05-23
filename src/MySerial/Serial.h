#ifndef SERIAL_H
#define SERIAL_H

#include <avr/io.h>

#include "../CircularList/CircularList.h"

class MySerial {
public:
    MySerial(uint32_t baudRate);

    bool available();

    uint8_t read();

    void print(const char* str);

    void println(const char* str);

    void ISR_RX();

    void ISR_UDRE();

    void ISR_TX();

    void ISR_FE();



    private:
        const int buffer_size = 128;
        CircularList<uint8_t> WriteBuffer = CircularList<uint8_t>(buffer_size);
        CircularList<uint8_t> ReadBuffer = CircularList<uint8_t>(buffer_size);

        void WriteFromBuffer();
};

#endif // SERIAL_H