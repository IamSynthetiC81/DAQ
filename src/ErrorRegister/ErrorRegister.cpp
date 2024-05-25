#include "ErrorRegister.h"

ErrorRegister::ErrorRegister(){
    ErrorRegister::reg = 0x00;    
}
ErrorRegister::ErrorRegister(uint8_t reg){
    ErrorRegister::reg = reg;
}

uint8_t ErrorRegister::SET(uint8_t bit){
    if (OutOfBounds(bit)) return;

    ErrorRegister::reg &= ~(1 << bit);

    return ErrorRegister::reg;
}

uint8_t ErrorRegister::SETREG(uint8_t word){
    uint8_t prevReg = reg;
    ErrorRegister::reg = word;

    return prevReg;
}

uint8_t ErrorRegister::CLEAR(uint8_t bit){
    if (OutOfBounds(bit)) return;

    uint8_t prevReg = reg;
    ErrorRegister::reg |= (1 << bit);

    return prevReg;
}

bool ErrorRegister::GET(uint8_t bit){
    if (OutOfBounds(bit)) return false;

    return ErrorRegister::reg &= (1 << bit);
}

uint8_t ErrorRegister::GET(){
    return ErrorRegister::reg;
}

bool ErrorRegister::OutOfBounds(uint8_t bit){
    return (bit > 7);
}

uint16_t ErrorRegister::capture_pc_value() {
    uint16_t pc_value;
    uint8_t sreg = SREG; // Save global interrupt flag
    // disable interrupts if they are enabled and save the status to restore it later
    if (sreg & 0x80) cli();

    asm volatile (
        "rcall get_pc \n\t" // Call label get_pc to push the return address (PC) onto the stack
        "get_pc: \n\t"
        "pop %A0 \n\t" // Pop lower byte of the return address (PC) into the lower byte of the result variable
        "pop %B0 \n\t" // Pop higher byte of the return address (PC) into the higher byte of the result variable
        : "=r" (pc_value) // Output
    );

    // enable interrupts if they were enabled before
    if (sreg & 0x80) sei();

    return pc_value;
};
