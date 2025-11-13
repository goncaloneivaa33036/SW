#include <avr/io.h>
#include <util/delay.h>

#define VREF 5.0 // Reference voltage in volts
#define ADC_MAX 1023.0 // Maximum ADC value for 10-bit ADC

// Initialize ADC
void adc_init() {
    // Set reference voltage to AVcc (5V)
    ADMUX = (1 << REFS0);
    // Enable ADC and set prescaler to 128 (16MHz/128 = 125kHz)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// Read ADC value from a given channel (0-7)
uint16_t adc_read(uint8_t channel) {
    // Select ADC channel (clear previous channel, set new one)
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    // Start conversion
    ADCSRA |= (1 << ADSC);
    // Wait for conversion to finish
    while (ADCSRA & (1 << ADSC));
    // Return ADC value
    return ADC;
}

// Convert ADC value to voltage
float adc_to_voltage(uint16_t adc_value) {
    return (adc_value / ADC_MAX) * VREF;
}