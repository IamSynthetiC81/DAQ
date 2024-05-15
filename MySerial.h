#include <avr/io.h>
#include <avr/interrupt.h>

// Buffer size for receiving data
#define BUFFER_SIZE 128

// Circular buffer for received data
volatile char rxBuffer[BUFFER_SIZE];
volatile uint8_t rxBufferHead = 0;
volatile uint8_t rxBufferTail = 0;

// Initialize the Serial Driver
void Serial_Init(uint32_t baudRate) {
    // Set baud rate
    uint16_t ubrr = F_CPU / 16 / baudRate - 1;
    UBRR0H = (ubrr >> 8);
    UBRR0L = (ubrr & 0xFF);

    
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);                       // Enable receiver and transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);                     // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSR0B |= (1 << RXCIE0);                                    // Enable receiver interrupt
}

// Transmit a single byte
void Serial_Write(char data) {
    if (rx)

    // Put data into buffer, sends the data
    UDR0 = data;
}

// Receive a single character
char Serial_Read() {
    // Wait for data to be received
    while (!(UCSR0A & (1 << RXC0)));

    // Get and return received data from buffer
    return UDR0;
}

// Check if data is available to read
bool Serial_Available() {
    return (rxBufferHead != rxBufferTail);
}

// Read a line of data from the buffer
void Serial_ReadLine(char* buffer, uint8_t bufferSize) {
    uint8_t i = 0;
    while (Serial_Available() && i < bufferSize - 1) {
        buffer[i] = Serial_Read();
        i++;
    }
    buffer[i] = '\0';
}

// Interrupt service routine for receiving data
ISR(USART0_RX_vect) {
    char data = UDR0;
    uint8_t nextHead = (rxBufferHead + 1) % BUFFER_SIZE;
    if (nextHead != rxBufferTail) {
        rxBuffer[rxBufferHead] = data;
        rxBufferHead = nextHead;
    }
}