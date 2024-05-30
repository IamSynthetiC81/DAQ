#ifndef ERROR_REGISTER_H
#define ERROR_REGISTER_H

#include <stdint.h>

class ErrorRegister {
    public:
        ErrorRegister();
        ErrorRegister(uint8_t reg);

        bool SET(uint8_t bit);
        uint8_t SETREG(uint8_t word);

        bool CLEAR(uint8_t bit);

        bool GET(uint8_t bit);
        uint8_t GET();

    private:
        uint8_t reg = 0x00;
        bool OutOfBounds(uint8_t bit);
};
#endif