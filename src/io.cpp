#include "io.h"
#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t pwm_counter = 0;
volatile uint8_t current_brightness = 0;
volatile uint8_t active_pin = 0;

// Timer0 overflow interrupt handler
ISR(TIMER0_OVF_vect) {
    if (pwm_counter >= 255) {
        pwm_counter = 0;
        PORTB |= (1 << active_pin); // Turn on LED at start of cycle
    }
    
    if (pwm_counter == current_brightness) {
        PORTB &= ~(1 << active_pin); // Turn off LED when counter reaches brightness level
    }
    
    pwm_counter++;
}

LED::LED(uint8_t pin) : pin(pin), brightness(255), state(false) {
    // Set the pin as output
    DDRB |= (1 << pin);
    active_pin = pin;
    
    // Configure Timer0 for PWM
    TCCR0A = 0; // Normal operation
    TCCR0B = (1 << CS01); // Set prescaler to 8
    TIMSK0 |= (1 << TOIE0); // Enable Timer0 overflow interrupt
    
    sei(); // Enable global interrupts
    off(); // Ensure LED is off initially
}

void LED::on() {
    state = true;
    current_brightness = brightness;
}

void LED::off() {
    state = false;
    current_brightness = 0;
    PORTB &= ~(1 << pin);
}

void LED::adjust_brightness(uint8_t new_brightness) {
    brightness = new_brightness;
    if (state) {
        current_brightness = brightness;
    }
}