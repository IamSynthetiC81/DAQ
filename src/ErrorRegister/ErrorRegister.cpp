#include "ErrorRegister.h"

ErrorRegister::ErrorRegister(){
    this->reg = 0x00;    
}
ErrorRegister::ErrorRegister(uint8_t reg){
    this->reg = reg;
}

bool ErrorRegister::SET(uint8_t bit){
    if (OutOfBounds(bit)) return false;;


    this->reg |= (1 << bit);

    return true;
}

uint8_t ErrorRegister::SETREG(uint8_t word){
    uint8_t prevReg = this->reg;
    this->reg = word;

    return prevReg;
}

bool ErrorRegister::CLEAR(uint8_t bit){
    if (OutOfBounds(bit)) return false;

    this->reg &= (~(1 << bit));

    return true;
}

bool ErrorRegister::GET(uint8_t bit){
    return this->reg & (1 << bit);
}

uint8_t ErrorRegister::GET(){
    return this->reg;
}

bool ErrorRegister::OutOfBounds(uint8_t bit){
    return (bit > 7);
}

