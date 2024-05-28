#ifndef ERROR_HANDLER_H
#define ERROR_HANDLER_H

#include "../ErrorRegister/ErrorRegister.h"
#include "../general/general.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>


// extern volatile ErrorRegister EREG_GEN;

class ErrorHandler {
    public:
        ErrorHandler(ErrorRegister* registers, size_t size);
    private:
        volatile ErrorRegister* VEC;
        size_t VEC_SIZE;

        bool OutOfBounds(uint8_t indx, uint8_t bit);

        int LINE = 0;
        char FILE[256] = "";
};

#endif