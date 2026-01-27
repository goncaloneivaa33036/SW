#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>

#define CAL_TENSAO 0.97   // calibração fina com multímetro

/* Tabela 0–9 (cátodo comum) */
uint8_t digitos_map[10] = {
    0b00111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110,
    0b01101101, 0b01111101, 0b00000111, 0b01111111, 0b01101111
};

/* Configuração via Serial */
uint8_t amostras_tensao = 64;
uint8_t amostras_corrente = 64;
uint16_t serial_delay_ms = 500;

/* ===== DISPLAY ===== */
void escreve_segmentos(uint8_t valor, uint8_t dp) {
    PORTD = (PORTD & 0x03) | ((valor & 0x3F) << 2); 
    if ((valor >> 6) & 1) PORTC |= (1 << PC2);
    else PORTC &= ~(1 << PC2);

    if(dp) PORTC |= (1 << PC3);
    else PORTC &= ~(1 << PC3);
}

void mostra_display(uint8_t bit_pino, uint8_t numero, uint8_t dp) {
    PORTB &= ~((1<<PB0)|(1<<PB1)|(1<<PB2));
    escreve_segmentos(digitos_map[numero], dp);
    PORTB |= (1<<bit_pino);
    _delay_ms(2);
}

/* ===== ADC ===== */
uint16_t adc_leitura(uint8_t canal, uint8_t n_amostras) {
    uint32_t soma = 0;
    ADMUX = (ADMUX & 0xF0) | (canal & 0x0F);

    for(uint8_t i=0; i<n_amostras; i++) {
        ADCSRA |= (1<<ADSC);
        while (ADCSRA & (1<<ADSC));
        soma += ADC;
    }
    return soma / n_amostras;
}

/* ===== SERIAL (AJUSTADO PARA 74880) ===== */
void serial_init(uint32_t baud) {
    uint16_t ubrr = (F_CPU / 16 / baud) - 1;

    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr;

    UCSR0B = (1<<TXEN0)|(1<<RXEN0);
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
}

void serial_write(char c) {
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = c;
}

void serial_print(const char* str) {
    while(*str) serial_write(*str++);
}

void serial_println(const char* str) {
    serial_print(str);
    serial_write('\r');
    serial_write('\n');
}

uint8_t serial_available() {
    return (UCSR0A & (1<<RXC0));
}

char serial_read() {
    return UDR0;
}

/* ===== PARSER SERIAL ===== */
void parse_command(char c) {
    static char buffer[8];
    static uint8_t idx = 0;

    if (c == '\n' || c == '\r') {
        buffer[idx] = 0;

        if (idx >= 2) {
            char cmd = buffer[0];
            int val = atoi(&buffer[1]);

            switch(cmd) {
                case 't':
                    amostras_tensao = val;
                    serial_println("Amostras tensao ajustadas");
                    break;

                case 'c':
                    amostras_corrente = val;
                    serial_println("Amostras corrente ajustadas");
                    break;

                case 'f':
                    serial_delay_ms = val;
                    serial_println("Frequencia serial ajustada");
                    break;
            }
        }
        idx = 0;
    }
    else {
        if (idx < sizeof(buffer)-1)
            buffer[idx++] = c;
    }
}

/* ===== MAIN ===== */
int main(void) {
    DDRD |= 0b11111100;
    DDRC |= (1<<PC2)|(1<<PC3)|(1<<PC4)|(1<<PC5);
    DDRB |= (1<<PB0)|(1<<PB1)|(1<<PB2);
    DDRB &= ~(1<<PB3);
    PORTB |= (1<<PB3);

    ADMUX  = (1<<REFS0);
    ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);

    serial_init(74880);

    uint8_t modo = 0;
    uint8_t btn_last = 1;
    uint16_t serial_counter = 0;

    while(1) {
        uint8_t btn_now = (PINB & (1<<PB3)) != 0;
        if (btn_last && !btn_now) {
            modo ^= 1;
            _delay_ms(200);
        }
        btn_last = btn_now;

        float valor_final = 0;

        if (modo == 0) {
            PORTC |= (1<<PC4);
            PORTC &= ~(1<<PC5);
            valor_final = adc_leitura(0, amostras_tensao) * 0.0293 * CAL_TENSAO;
        }
        else {
            PORTC |= (1<<PC5);
            PORTC &= ~(1<<PC4);
            uint16_t leitura = adc_leitura(1, amostras_corrente);
            float v = (leitura * 5.0) / 1024.0;
            valor_final = fabsf((v - 2.5) / 0.185);
            if (valor_final < 0.05) valor_final = 0;
        }

        uint16_t n = (uint16_t)(valor_final * 10 + 0.5);
        if (n > 999) n = 999;

        uint8_t d[3] = { n/100, (n/10)%10, n%10 };

        for(uint8_t i=0;i<10;i++) {
            mostra_display(PB0, d[0], 0);
            mostra_display(PB1, d[1], 1);
            mostra_display(PB2, d[2], 0);
        }

        serial_counter += 10;
        if (serial_counter >= serial_delay_ms) {

            // ===== DEBUG DO ADC =====
            uint16_t adc_raw = adc_leitura(0, amostras_tensao);
            char debug_adc[40];
            sprintf(debug_adc, "ADC bruto: %u", adc_raw);
            serial_println(debug_adc);

            // ===== DEBUG DO VALOR CALCULADO =====
            char debug_val[40];
            sprintf(debug_val, "Valor calc: %.4f", valor_final);
            serial_println(debug_val);

            // ===== ENVIO NORMAL =====
            if (isnan(valor_final) || isinf(valor_final)) {
                serial_println("Valor invalido");
            } else {
                char buf[80];
                sprintf(buf, "Modo %s: %.2f",
                        (modo==0?"Tensao":"Corrente"), valor_final);
                serial_println(buf);
            }

            serial_counter = 0;
        }

        while(serial_available()) {
            parse_command(serial_read());
        }
    }
}
