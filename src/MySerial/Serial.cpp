#include "Serial.h"
#include <stdio.h>
#include <avr/interrupt.h>

CircularList<uint8_t> WriteBuffer(128);
CircularList<uint8_t> ReadBuffer(128);

MySerial::MySerial(uint32_t baudRate){
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

void MySerial::print(const char* data) {
    while (*data) {
        WriteBuffer.push(*data++);
    }

    // Enable TX complete interrupt
    UCSR0B |= (1 << TXCIE0);

    // Write to buffer
    WriteFromBuffer();  // Call the function to start writing immediately if buffer was empty
}

void MySerial::print(char data) {
    WriteBuffer.push(data);

    // Enable TX complete interrupt
    UCSR0B |= (1 << TXCIE0);

    // Write to buffer
    WriteFromBuffer();  // Call the function to start writing immediately if buffer was empty
}

void MySerial::print(uint8_t data) {
    WriteBuffer.push(data);

    // Enable TX complete interrupt
    UCSR0B |= (1 << TXCIE0);

    // Write to buffer
    WriteFromBuffer();  // Call the function to start writing immediately if buffer was empty
}

void MySerial::print(uint16_t data) {
    WriteBuffer.push(data >> 8);
    WriteBuffer.push(data);

    // Enable TX complete interrupt
    UCSR0B |= (1 << TXCIE0);

    // Write to buffer
    WriteFromBuffer();  // Call the function to start writing immediately if buffer was empty
}

void MySerial::print(uint32_t data) {
    WriteBuffer.push(data >> 24);
    WriteBuffer.push(data >> 16);
    WriteBuffer.push(data >> 8);
    WriteBuffer.push(data);

    // Enable TX complete interrupt
    UCSR0B |= (1 << TXCIE0);

    // Write to buffer
    WriteFromBuffer();  // Call the function to start writing immediately if buffer was empty
}

void MySerial::print(int8_t data) {
    WriteBuffer.push(data);

    // Enable TX complete interrupt
    UCSR0B |= (1 << TXCIE0);

    // Write to buffer
    WriteFromBuffer();  // Call the function to start writing immediately if buffer was empty
}

void MySerial::print(int16_t data) {
    WriteBuffer.push(data >> 8);
    WriteBuffer.push(data);

    // Enable TX complete interrupt
    UCSR0B |= (1 << TXCIE0);

    // Write to buffer
    WriteFromBuffer();  // Call the function to start writing immediately if buffer was empty
}

void MySerial::print(int32_t data) {
    WriteBuffer.push(data >> 24);
    WriteBuffer.push(data >> 16);
    WriteBuffer.push(data >> 8);
    WriteBuffer.push(data);

    // Enable TX complete interrupt
    UCSR0B |= (1 << TXCIE0);

    // Write to buffer
    WriteFromBuffer();  // Call the function to start writing immediately if buffer was empty
}

void MySerial::print(float data) {
    // Convert float to string
    char buffer[32];
    sprintf(buffer, "%f", data);

    // Print string
    print(buffer);
}

void MySerial::print(double data) {
    // Convert double to string
    char buffer[32];
    sprintf(buffer, "%f", data);

    // Print string
    print(buffer);
}

void MySerial::println(const char* data) {
    // Print data
    print(data);

    // Print newline
    print("\r\n");
}

void MySerial::println(char data) {
    // Print data
    print(data);

    // Print newline
    print("\r\n");
}

void MySerial::println(uint8_t data) {
    // Print data
    print(data);

    // Print newline
    print("\r\n");
}

void MySerial::println(uint16_t data) {
    // Print data
    print(data);

    // Print newline
    print("\r\n");
}

void MySerial::println(uint32_t data) {
    // Print data
    print(data);

    // Print newline
    print("\r\n");
}

void MySerial::println(int8_t data) {
    // Print data
    print(data);

    // Print newline
    print("\r\n");
}

void MySerial::println(int16_t data) {
    // Print data
    print(data);

    // Print newline
    print("\r\n");
}

void MySerial::println(int32_t data) {
    // Print data
    print(data);

    // Print newline
    print("\r\n");
}

void MySerial::println(float data) {
    // Print data
    print(data);

    // Print newline
    print("\r\n");
}
void MySerial::println(double data) {
    // Print data
    print(data);

    // Print newline
    print("\r\n");
}

int MySerial::ReadBufferToChar(char* buffer){
    int i = 0;
    while (ReadBuffer.getSize() > 0) {
        buffer[i] = ReadBuffer.pop();
        i++;
    }
    return i;
}


bool MySerial::available(){
    // Check if data is available to read
    return (UCSR0A & (1 << RXC0));
}

uint8_t MySerial::read(){
    // Wait for data to be received
    while (!(UCSR0A & (1 << RXC0)));

    // Get and return received data from buffer
    return UDR0;
}

// void ISR_RX(){
//     ReadBuffer.push(UDR0);
// }

// void ISR_TX(){
//     if (WriteBuffer.getSize() > 0) {
//         UDR0 = WriteBuffer.pop();
//     } else {
//         // Disable TX complete interrupt
//         UCSR0B &= ~(1 << TXCIE0);
//     }
// }

void MySerial::WriteFromBuffer(){
    if (WriteBuffer.getSize() > 0) {
        UDR0 = WriteBuffer.pop();
    } else {
        // Disable TX complete interrupt
        UCSR0B &= ~(1 << TXCIE0);
    }
}



ISR(USART_RX_vect) {
    ReadBuffer.push(UDR0);
}

ISR(USART_TX_vect) {
    if (WriteBuffer.getSize() > 0) {
        UDR0 = WriteBuffer.pop();
    } else {
        // Disable TX complete interrupt
        UCSR0B &= ~(1 << TXCIE0);
    }
}