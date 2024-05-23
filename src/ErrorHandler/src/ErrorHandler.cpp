#include "ErrorHandler.h"
#include <stdlib.h>
#include <stdint.h>

// volatile ErrorRegister EREG_GEN = ErrorRegister(nullptr);

ErrorHandler::ErrorHandler(ErrorRegister* registers, size_t size){
    assert(registers != nullptr);
    assert (size > 0);
    assert(size == sizeof(registers)/sizeof(registers[0]));

    ErrorHandler::VEC = registers;
    ErrorHandler::VEC_SIZE = size;
}

