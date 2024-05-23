#include "Serial.h"



MySerial::MySerial(uint32_t baudRate) {
    // Set baud rate
    uint16_t ubrr = F_CPU / 16 / baudRate - 1;
    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr;

    // Enable receiver and transmitter
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);

    // Set frame format: 8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    // Enable RX complete interrupt
    UCSR0B |= (1 << RXCIE0);
}

bool MySerial::available() {
    // Check if data is available to read
    return (UCSR0A & (1 << RXC0));
}

uint8_t MySerial::read() {
    // Wait for data to be received
    while (!(UCSR0A & (1 << RXC0)));

    // Get and return received data from buffer
    return UDR0;
}

void MySerial::print(const char* str) {
    for (int i = 0; str[i] != '\0'; i++) {
        WriteBuffer.push(str[i]);
    }
    
    // Enable TX complete interrupt
    UCSR0B |= (1 << TXCIE0);

    // Write to buffer
    UDR0 = WriteBuffer.pop();
}

void MySerial::println(const char* str) {
    print(str);
    print("\n");
}

void MySerial::ISR_RX() {
    ReadBuffer.push(UDR0);
}


void MySerial::ISR_TX() {
    if (WriteBuffer.getSize() > 0) {
        UDR0 = WriteBuffer.pop();
    } else {
        // Disable TX complete interrupt
        UCSR0B &= ~(1 << TXCIE0);
    }
}
