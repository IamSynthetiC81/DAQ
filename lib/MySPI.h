#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>

class SPIDriver {
public:
    SPIDriver() {
        DDRB |= (1 << DDB2) | (1 << DDB1) | (1 << DDB0);                // Set MOSI, SCK, and SS as output pins
        SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);                  // Enable SPI, Master mode, Fosc/16
        SPCR |= (1 << SPIE);                                            // Enable SPI interrupt

        txData = new uint8_t[BUFFER_SIZE];                              // Allocate memory for the buffer
        txLength = 0;                                                   // Initialize buffer length
        
        sei();                                                          // Enable global interrupts                               
    }
    

    /**
     * Write data to the SPI bus
     * @param data The data to write
     * @param length The length of the data
     * 
     * @returns EXIT_SUCCESS if the data was written successfully, EXIT_FAILURE otherwise
    */
    int writeData(const uint8_t* data, const uint16_t length) {
        if (txLength + length > BUFFER_SIZE) return EXIT_FAILURE;       // Buffer overflow

        memccpy((void*)txData,(void*)data,'\0', length);                // Copy data to buffer                  
        txLength += length;                                             // Update buffer length
        
        SPDR = *txData;                                                 // Transmit the first byte
        
        txData++;                                                       // Move to the next byte
        txLength--;                                                     // Update buffer length
        
        return EXIT_SUCCESS;                                            // Data written successfully
    }
    
    bool isBusy() {
        return txLength > 0;
    }

    void transmitNextByte() {
        if (txLength > 0) {
            SPDR = *txData;                                             // Transmit the next byte
            txData++;                                                   // Move to the next byte
            txLength--;                                                 // Update buffer length
        }
    }
    
private:
    const uint16_t BUFFER_SIZE = 256;
    uint8_t *txData;
    uint16_t txLength;
};

// Create an instance of the SPI driver
SPIDriver spiDriver;

// SPI interrupt service routine
ISR(SPI_STC_vect) {
    spiDriver.transmitNextByte();                                       // Transmit the next byte
}