#ifndef ERROR_REGISTER_H
#define ERROR_REGISTER_H

#include <stdint.h>
#include <avr/interrupt.h>


class ErrorRegister {
    public:
        ErrorRegister();
        ErrorRegister(uint8_t reg);

        uint8_t SET(uint8_t bit);
        uint8_t SETREG(uint8_t register);

        uint8_t CLEAR(uint8_t bit);

        bool GET(uint8_t bit);
        uint8_t GET();

    private:
        uint8_t reg = 0x00;

        bool OutOfBounds(uint8_t bit);

        uint16_t capture_pc_value();
};
#endif