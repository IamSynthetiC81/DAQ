#define F_CPU 16000000UL
#define BAUD 115200
#define MYUBRR F_CPU/16/BAUD-1

#include <avr/io.h>
#include <avr/interrupt.h>

volatile char *currentChar;

void USART_Init(unsigned int ubrr) {
    // Set baud rate
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    // Enable receiver and transmitter
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    // Set frame format: 8data, 2stop bit
    UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

void USART_Transmit_NonBlocking(const char *data) {
    currentChar = data;
    // Enable UDRE interrupt
    UCSR0B |= (1<<UDRIE0);
}

// ISR(USART_UDRE_vect) {
//     if (*currentChar) {
//         // Put data into buffer, sends the data
//         UDR0 = *currentChar++;
//     } else {
//         // Disable UDRE interrupt
//         UCSR0B &= ~(1<<UDRIE0);
//     }
// }

int main(void) {
    USART_Init(MYUBRR);
    sei(); // Enable global interrupts
    while(1) {
        USART_Transmit_NonBlocking("Hello, world!");
        _delay_ms(1000);
    }
}