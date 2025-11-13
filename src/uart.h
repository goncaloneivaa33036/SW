#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Initialize UART
void uart_init(unsigned int baud) {
  unsigned int ubrr = F_CPU / 16 / baud - 1;
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  UCSR0B = (1 << TXEN0); // Enable transmitter
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 data bits, 1 stop bit
}

// Transmit a single character via UART
void uart_transmit(unsigned char data) {
  while (!(UCSR0A & (1 << UDRE0))); // Wait for empty transmit buffer
  UDR0 = data;
}

// Transmit a string via UART
void uart_transmit_string(const char* str) {
  while (*str) {
    uart_transmit(*str++);
  }
}

void uart_new_line() {
    uart_transmit('\r');
    uart_transmit('\n');
}

void float_to_string(float value, char* buffer, size_t buffer_size, int decimal_places) {
    if (buffer_size < 2) {
        if (buffer_size == 1) {
            buffer[0] = '\0'; // Ensure null-termination
        }
        return; // Not enough space
    }

    // Handle negative values
    if (value < 0) {
        if (buffer_size < 3) { // Need space for '-', digit, and '\0'
            buffer[0] = '\0'; // Ensure null-termination
            return; // Not enough space
        }
        *buffer++ = '-';
        buffer_size--;
        value = -value;
    }

    // Extract integer part
    int int_part = (int)value;
    float frac_part = value - int_part;

    // Convert integer part to string
    char int_buffer[20]; // Temporary buffer for integer part
    itoa(int_part, int_buffer, 10);
    size_t int_len = strlen(int_buffer);

    if (int_len + 1 > buffer_size) { // +1 for null terminator
        buffer[0] = '\0'; // Ensure null-termination
        return; // Not enough space
    }

    strcpy(buffer, int_buffer);
    buffer += int_len;
    buffer_size -= int_len;

    // Handle fractional part
    if (decimal_places > 0) {
        if (buffer_size < 2) { // Need space for '.' and at least one digit
            buffer[0] = '\0'; // Ensure null-termination
            return; // Not enough space
        }
        *buffer++ = '.';
        buffer_size--;

        for (int i = 0; i < decimal_places; i++) {
            frac_part *= 10;
            int digit = (int)frac_part;
            if (buffer_size < 2) { // Need space for digit and null terminator
                buffer[0] = '\0'; // Ensure null-termination
                return; // Not enough space
            }
            *buffer++ = '0' + digit;
            buffer_size--;
            frac_part -= digit;
        }
    }

    *buffer = '\0'; // Null-terminate the string
}