#include "ErrorHandler.h"
#include <stdlib.h>
#include <stdint.h>

// volatile ErrorRegister EREG_GEN = ErrorRegister(nullptr);

ErrorHandler::ErrorHandler(ErrorRegister* registers, size_t size){
    if(registers == nullptr) {
        registers = (ErrorRegister*)malloc(size * sizeof(ErrorRegister));
        for (size_t i = 0; i < size; i++) {
            registers[i] = ErrorRegister();
        }
    };
    if(size == 0){
        size = sizeof(registers)/sizeof(registers[0]);
    }
    if(size != sizeof(registers)/sizeof(registers[0])){
        // Error
    }

    ErrorHandler::VEC = registers;
    ErrorHandler::VEC_SIZE = size;
}

