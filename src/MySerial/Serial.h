#pragma once

#include <stdint.h>
#include <avr/io.h>

#include "../CircularList/CircularList.h"

extern CircularList<uint8_t> WriteBuffer;
extern CircularList<uint8_t> ReadBuffer;

extern volatile bool newLine;

class MySerial {
public:
    MySerial(uint32_t baudRate);
    bool available();

    uint8_t read();
    void print(const char* data);
    void print(char data);
    void print(uint8_t data);
    void print(uint16_t data);
    void print(uint32_t data);
    void print(int8_t data);
    void print(int16_t data);
    void print(int32_t data);
    void print(float data);
    void print(double data);

    void println(const char* data);
    void println(char data);
    void println(uint8_t data);
    void println(uint16_t data);
    void println(uint32_t data);
    void println(int8_t data);
    void println(int16_t data);
    void println(int32_t data);
    void println(float data);
    void println(double data);
  
    int ReadBufferToChar(char* buffer);

private:
    void WriteFromBuffer();
};