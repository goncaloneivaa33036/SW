// ================================================================
//
//  Funcionalidades incluídas:
//   - Medição de Tensão (com divisor e referência interna 5V)
//   - Medição de Corrente usando o sensor ACS712-05B
//   - Display de 3 dígitos multiplexado (cátodo comum)
//   - UART para envio de dados ao computador via FTDI/USB
//   - Botão para alternar entre modos (Tensão ↔ Corrente)
//   - LED indicador de thresholds
//
//  
// ================================================================

#include <Arduino.h>
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

// ================================================================
//  CONSTANTES DO SENSOR DE CORRENTE ACS712-05B
// ================================================================
#define ACS712_SENSITIVITY 0.185f   // 185 mV/A
#define ACS712_OFFSET_V    2.5f     // Offset a 0 A

// ================================================================
//  DEFINIÇÃO DE PINOS
// ================================================================
#define BTN_PIN PD2      // Botão
#define LED_PIN PB5      // LED indicador
#define ADC_CHANNEL 0    // ADC0 (A0)

// ================================================================
//  DISPLAY 7 SEGMENTOS (CÁTADO COMUM)
// ================================================================
// Segmentos a–f → PD2..PD7
// Segmento g → PB0
// Ponto decimal → PB1
// Dígitos → PB2, PB3, PB4

#define SEG_DDR  DDRD
#define SEG_PORT PORTD
#define DIG_DDR  DDRB
#define DIG_PORT PORTB
#define DIG1 PB2
#define DIG2 PB3
#define DIG3 PB4
#define SEG_G  PB0
#define SEG_DP PB1

// ================================================================
//  DIVISOR DE TENSÃO
//  (Exemplo: 30 V → 5 V)
// ================================================================
#define DIVIDER_FACTOR 6.0f   // Ajustar aos valores reais das resistências

// ================================================================
//  TABELA DE SEGMENTOS (0–9)
// ================================================================
const uint8_t seg7[10] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F  // 9
};

// ================================================================
//  UART
// ================================================================
void uart_init(unsigned int baud) {
    unsigned int ubrr = (F_CPU / (16UL * baud)) - 1;
    UBRR0H = ubrr >> 8;
    UBRR0L = ubrr;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_tx(char c) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void uart_print(const char *s) {
    while (*s) uart_tx(*s++);
}

void uart_nl(void) {
    uart_tx('\r');
    uart_tx('\n');
}

// ================================================================
//  ADC – SEMPRE A 5 V (AVcc)
// ================================================================
void adc_init(void) {
    ADMUX = (1 << REFS0); // Referência AVcc = 5 V
    ADCSRA = (1 << ADEN)  // Ativar ADC
           | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler 128
}

uint16_t adc_read(uint8_t ch) {
    ADMUX = (ADMUX & 0xF8) | ch;
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

// Converter ADC para volts (5 V)
float adc_to_volts(uint16_t adc) {
    return (adc * 5.0f) / 1023.0f;
}

// ================================================================
//  FUNÇÕES DO DISPLAY
// ================================================================
void digit_on(uint8_t d)  { DIG_PORT |=  (1 << d); }
void digit_off(uint8_t d) { DIG_PORT &= ~(1 << d); }

void set_segments(uint8_t val, uint8_t dp) {
    // Preservar RX/TX (PD0, PD1)
    uint8_t keep = PORTD & 0x03;
    PORTD = keep | ((seg7[val] & 0x3F) << 2);

    if (seg7[val] & (1 << 6)) DIG_PORT |=  (1 << SEG_G);
    else                      DIG_PORT &= ~(1 << SEG_G);

    if (dp) DIG_PORT |=  (1 << SEG_DP);
    else    DIG_PORT &= ~(1 << SEG_DP);
}

void display_number(int value10) {
    if (value10 < 0) value10 = 0;
    if (value10 > 999) value10 = 999;

    uint8_t d1 = value10 / 100;
    uint8_t d2 = (value10 / 10) % 10;
    uint8_t d3 = value10 % 10;

    for (uint8_t i = 0; i < 5; i++) {
        digit_off(DIG1); digit_off(DIG2); digit_off(DIG3);

        set_segments(d1, 0);
        digit_on(DIG1); _delay_ms(2); digit_off(DIG1);

        set_segments(d2, 1);
        digit_on(DIG2); _delay_ms(2); digit_off(DIG2);

        set_segments(d3, 0);
        digit_on(DIG3); _delay_ms(2); digit_off(DIG3);
    }
}

// ================================================================
//  MAIN
// ================================================================
int main(void) {

    uart_init(9600);
    adc_init();
    DDRB |= (1 << LED_PIN);
        DDRD &= ~(1 << BTN_PIN);
    PORTD |= (1 << BTN_PIN);
        SEG_DDR |= 0xFC;
    DIG_DDR |= (1 << DIG1) | (1 << DIG2) | (1 << DIG3)
             | (1 << SEG_G) | (1 << SEG_DP);

    uint8_t modo = 0;
    uint8_t last_btn = 1;
    char txt[64];

    uart_print("Modo inicial: Tensao");
    uart_nl();

    while (1) {

        uint8_t btn = (PIND & (1 << BTN_PIN)) ? 1 : 0;
        if (last_btn && !btn) {
            modo = !modo;
            uart_print(modo ? "Modo: Corrente" : "Modo: Tensao");
            uart_nl();
            _delay_ms(200);
        }
        last_btn = btn;

        uint16_t adc = adc_read(ADC_CHANNEL);
        float v_adc = adc_to_volts(adc);

        if (!modo) {
            // ---------------- TENSÃO ----------------
            float vin = v_adc * DIVIDER_FACTOR;
            snprintf(txt, sizeof(txt), "Tensao: %.2f V", vin);
            uart_print(txt);
            uart_nl();

            display_number((int)(vin * 10));

            if (vin > 15.0f) PORTB |= (1 << LED_PIN);
            else             PORTB &= ~(1 << LED_PIN);

        } else {
            // ---------------- CORRENTE ----------------
            float corrente = (v_adc - ACS712_OFFSET_V) / ACS712_SENSITIVITY;
            snprintf(txt, sizeof(txt), "Corrente: %.3f A", corrente);
            uart_print(txt);
            uart_nl();

            int mA = (int)(fabs(corrente) * 1000);
            display_number(mA);

            if (fabs(corrente) > 1.5f) PORTB |= (1 << LED_PIN);
            else                       PORTB &= ~(1 << LED_PIN);
        }

        _delay_ms(200);
    }
    
    return 0;
}

                                                                     