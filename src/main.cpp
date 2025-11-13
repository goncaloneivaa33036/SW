#include <Arduino.h>
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>

#define R_SHUNT 100.0   // Resistência usada para medir corrente (em ohms)
#define BTN_PIN PD2      // Pino do botão (D2 no Arduino)
#define LED_PIN PB5      // Pino do LED (13 no Arduino)

// ---------- UART ----------
void uart_init(unsigned int baud) {
    unsigned int ubrr = (F_CPU / (16UL * baud)) - 1;
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_transmit(char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void uart_transmit_string(const char *str) {
    while (*str) uart_transmit(*str++);
}

void uart_new_line() {
    uart_transmit('\r');
    uart_transmit('\n');
}

// ---------- ADC ----------
void adc_init() {
    ADMUX = (1 << REFS0); // AVcc como referência
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(uint8_t ch) {
    ch &= 0x07;
    ADMUX = (ADMUX & 0xF8) | ch;
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

uint32_t adc_to_millivolts(uint16_t adc_value) {
    return ((uint32_t)adc_value * 5000UL) / 1023UL;
}

// ---------- Programa principal ----------
int main(void) {
    uart_init(9600);
    adc_init();

    DDRB |= (1 << LED_PIN);   // LED como saída
    DDRD &= ~(1 << BTN_PIN);  // Botão como entrada
    PORTD |= (1 << BTN_PIN);  // Ativa pull-up interno no botão (ligar botão ao GND)

    uint8_t modo = 0; // 0 = tensão, 1 = corrente
    char buffer[64];
    uint8_t last_btn_state = 1; // Começa em 1 (botão não pressionado)

    // --- Modo inicial: medir tensão ---
    uart_transmit_string("Modo inicial: Tensao");
    uart_new_line();

    while (1) {
        // Leitura do botão (detetar transição)
        uint8_t btn_state = (PIND & (1 << BTN_PIN)) ? 1 : 0;
        if (last_btn_state == 1 && btn_state == 0) { // botão pressionado
            modo = !modo; // alterna entre tensão e corrente
            if (modo == 0)
                uart_transmit_string("Modo alterado para: Tensao");
            else
                uart_transmit_string("Modo alterado para: Corrente");
            uart_new_line();
            _delay_ms(200); // debounce simples
        }
        last_btn_state = btn_state;

        // Lê o valor do ADC (A0)
        uint16_t adc_value = adc_read(0);
        uint32_t mv = adc_to_millivolts(adc_value);

        if (modo == 0) {
            // ---- MODO TENSÃO ----
            uint16_t volts = mv / 1000;
            uint16_t centi = (mv % 1000) / 10;
            snprintf(buffer, sizeof(buffer), "ADC: %u -> %u.%02u V", adc_value, volts, centi);
        } else {
            // ---- MODO CORRENTE ----
            float corrente = (mv / 1000.0) / R_SHUNT; // I = V / R
            snprintf(buffer, sizeof(buffer), "ADC: %u -> %.2f mA", adc_value, corrente * 1000);
        }

        uart_transmit_string(buffer);
        uart_new_line();

        // LED ON se valor > metade da escala
        if (mv > 2500)
            PORTB |= (1 << LED_PIN);
        else
            PORTB &= ~(1 << LED_PIN);

        _delay_ms(500);
    }
}


                                                                         