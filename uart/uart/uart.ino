#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 16000000UL    // AVR clock frequency in Hz, used by util/delay.h
#define BAUD 115200
#define BAUDRATE ((F_CPU)/(BAUD*8UL)-1)

volatile unsigned char rx_complete_flg; //clear when data is transmitted and set when data is received    
unsigned char rd_data_flg; //avoids char reception in ISR until first char in polling is transmitted
unsigned char buff; //stores char to be transmitted by polling
unsigned char data; //stores further received char from ISR

void usart_init(void);
void usart_tx(unsigned char msg);
unsigned char usart_rx(void);

int main(void) {
    usart_init();
    while (1) {
        if (rx_complete_flg) {
            rx_complete_flg = 0;
            usart_tx(data);
        }
    }
}

void usart_init() {
    SREG |= 0x80; //set global interrupt enable bit
    UCSR0A |= (1 << U2X0);   // double transmission speed  
    UCSR0B |= (1 << RXCIE0) | (1 << TXEN0) | (1 << RXEN0); //enable transmitter, receiver, enable receive complete interrupt       
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); /* 8 data bits, 1 stop bit */

    UBRR0H = (BAUDRATE>>8);  // shift the register right by 8 bits to get the upper 8 bits
    UBRR0L = BAUDRATE;       // set baud rate
}

void usart_tx(unsigned char msg) {
    while (!(UCSR0A & (1 << UDRE0))); //wait until transmit buffer becomes empty 
    UDR0 = data;
}

unsigned char usart_rx() {
    while (!(UCSR0A & (1 << RXC0))); //wait until reception is completed
    return UDR0; //receive first character by polling
}

ISR(USART0_RX_vect) {
    rx_complete_flg = 1;

    if (rd_data_flg){
        data = UDR0;    
    }else rd_data_flg = 1; //set flag to start reading data from ISR
}
