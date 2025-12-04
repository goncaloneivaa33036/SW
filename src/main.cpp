// ================================================================
//  PROJETO DE MULTÍMETRO DIGITAL – Código Final Completo
//  Comentado em Português de Portugal
//
//  Funcionalidades incluídas:
//   - Medição de Tensão (com divisor e referência interna 1.1V)
//   - Medição de Corrente usando o sensor ACS712-05B
//   - Display de 3 dígitos multiplexado (cátodo comum)
//   - UART para envio de dados ao computador via FTDI/USB
//   - Botão para alternar entre modos (Tensão ↔ Corrente)
//   - LED indicador de thresholds
//
//  NOTA: Os comentários explicam cada linha para facilitar avaliação
// ================================================================

#include <Arduino.h>
#define F_CPU 16000000UL       // Frequência do ATmega328P (16 MHz)
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>

// =====================================================================
//  CONSTANTES DO SENSOR DE CORRENTE ACS712-05B
// =====================================================================
// Sensibilidade do modelo ACS712ELCTR-05B-T = 185 mV/A
#define ACS712_SENSITIVITY 0.185f
// Offset do sensor: quando a corrente é 0 A, a saída é ~2.5V
#define ACS712_OFFSET_V    2.5f

// =====================================================================
//  DEFINIÇÃO DE PINOS
// =====================================================================
#define BTN_PIN PD2   // Botão no pino PD2
#define LED_PIN PB5   // LED no pino PB5 (equivalente ao LED da Arduino Uno)
#define ADC_CHANNEL A0 // Leitura analógica no pino ADC0 (PC0)

// =====================================================================
//  CONFIGURAÇÃO DOS SEGMENTOS DO DISPLAY
// =====================================================================
// Segmentos principais (a..f) → PD2..PD7
// Segmento g → PB0
// Ponto decimal (dp) → PB1
// Dígitos (cátodo comum) → PB2, PB3, PB4
#define SEG_PORT PORTD
#define SEG_DDR  DDRD
#define DIG_PORT PORTB
#define DIG_DDR  DDRB

#define DIG1_PIN PB2   // Dígito 1 (esquerda)
#define DIG2_PIN PB3   // Dígito 2 (meio)
#define DIG3_PIN PB4   // Dígito 3 (direita)

#define SEG_G_PIN PB0  // Segmento 'g'
#define SEG_DP_PIN PB1 // Segmento 'dp' (ponto decimal)

// Fator do divisor de tensão (exemplo: 270k e 10k → fator = 28)
#define DIVIDER_FACTOR 28.0f

// =====================================================================
//  TABELA DE SEGMENTOS PARA OS DÍGITOS 0..9
// =====================================================================
// Formato: bit0=a ... bit6=g (bit7 seria o DP, tratado à parte)
const uint8_t seg7_basic[10] = {
    0x3F, // 0 → a b c d e f
    0x06, // 1 → b c
    0x5B, // 2 → a b d e g
    0x4F, // 3 → a b c d g
    0x66, // 4 → b c f g
    0x6D, // 5 → a c d f g
    0x7D, // 6 → a c d e f g
    0x07, // 7 → a b c
    0x7F, // 8 → todos os segmentos
    0x6F  // 9 → a b c d f g
};

// =====================================================================
//  FUNÇÕES PARA CONTROLO DOS DÍGITOS DO DISPLAY
// =====================================================================
static inline void digit_on(uint8_t d) {
    if (d == 1) DIG_PORT |= (1 << DIG1_PIN);
    else if (d == 2) DIG_PORT |= (1 << DIG2_PIN);
    else if (d == 3) DIG_PORT |= (1 << DIG3_PIN);
}

static inline void digit_off(uint8_t d) {
    if (d == 1) DIG_PORT &= ~(1 << DIG1_PIN);
    else if (d == 2) DIG_PORT &= ~(1 << DIG2_PIN);
    else if (d == 3) DIG_PORT &= ~(1 << DIG3_PIN);
}

// =====================================================================
//  ESCREVER SEGMENTOS NO DISPLAY
// =====================================================================
void write_segments(uint8_t pattern7, uint8_t dp) {

    // Guardar PD0/PD1 (UART) e só modificar PD2..PD7
    uint8_t pd_keep = PORTD & 0x03;

    // a..f → PD2..PD7
    uint8_t a_to_f = (pattern7 & 0x3F) << 2;
    PORTD = pd_keep | a_to_f;

    // Segmento g → PB0
    if (pattern7 & (1 << 6)) DIG_PORT |= (1 << SEG_G_PIN);
    else DIG_PORT &= ~(1 << SEG_G_PIN);

    // DP → PB1
    if (dp) DIG_PORT |= (1 << SEG_DP_PIN);
    else DIG_PORT &= ~(1 << SEG_DP_PIN);
}

// =====================================================================
//  UART – Comunicação com o PC
// =====================================================================
void uart_init(unsigned int baud) {
    unsigned int ubrr = (F_CPU / (16UL * baud)) - 1;
    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr;
    UCSR0B = (1 << TXEN0);                      // habilitar TX
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);     // 8 bits
}

void uart_transmit(char data) {
    while (!(UCSR0A & (1 << UDRE0)));  // esperar buffer
    UDR0 = data;                       // enviar byte
}

void uart_transmit_string(const char *str) {
    while (*str) uart_transmit(*str++);
}

void uart_new_line() {
    uart_transmit('\r');
    uart_transmit('\n');
}

// =====================================================================
//  ADC – Inicialização e Leitura
// =====================================================================
void adc_enable() { ADCSRA |= (1 << ADEN); }
void adc_disable() { ADCSRA &= ~(1 << ADEN); }

// Selecionar referência ADC:
// 0 = AVcc (5V)
// 1 = interna 1.1V
void adc_set_ref_internal1v1(uint8_t use_internal) {
    if (use_internal) {
        ADMUX |= (1 << REFS1) | (1 << REFS0);
    } else {
        ADMUX &= ~(1 << REFS1);
        ADMUX |= (1 << REFS0);
    }
}

uint16_t adc_read_ch(uint8_t ch) {
    ADMUX = (ADMUX & 0xF8) | (ch & 0x07);
    ADCSRA |= (1 << ADSC);           // iniciar conversão
    while (ADCSRA & (1 << ADSC));    // esperar fim
    return ADC;
}

float adc_to_volts(uint16_t adc, uint8_t internal_ref) {
    return internal_ref ?
        ((float)adc * 1.1f / 1023.0f) :   // ref 1.1V
        ((float)adc * 5.0f / 1023.0f);    // ref 5V
}

// =====================================================================
//  FUNÇÃO DE MULTIPLEXAÇÃO DO DISPLAY (3 dígitos)
// =====================================================================
void display_show_scaled_int(int value_scaled_by_10) {

    // Garantir limites 0..999
    if (value_scaled_by_10 < 0) value_scaled_by_10 = 0;
    if (value_scaled_by_10 > 999) value_scaled_by_10 = 999;

    uint8_t d3 = value_scaled_by_10 % 10;
    uint8_t d2 = (value_scaled_by_10 / 10) % 10;
    uint8_t d1 = (value_scaled_by_10 / 100) % 10;

    // Multiplexar 6 ciclos por chamada
    for (uint8_t k = 0; k < 6; k++) {

        // Dígito 1 (sem decimal)
        digit_off(1); digit_off(2); digit_off(3);
        write_segments(seg7_basic[d1], 0);
        digit_on(1);
        _delay_ms(2);
        digit_off(1);

        // Dígito 2 (com decimal)
        write_segments(seg7_basic[d2], 1);
        digit_on(2);
        _delay_ms(2);
        digit_off(2);

        // Dígito 3 (sem decimal)
        write_segments(seg7_basic[d3], 0);
        digit_on(3);
        _delay_ms(2);
        digit_off(3);
    }
}

// =====================================================================
//  PROGRAMA PRINCIPAL
// =====================================================================
int main(void) {

    uart_init(9600);     // iniciar comunicação série
    adc_enable();        // iniciar ADC

    // LED como saída
    DDRB |= (1 << LED_PIN);

    // Botão como entrada → com pull-up
    DDRD &= ~(1 << BTN_PIN);
    PORTD |= (1 << BTN_PIN);

    // Configurar segmentos: PD2..PD7 como saída
    SEG_DDR |= 0xFC;

    // Configurar PB0..PB4 como saída (g, dp, dígitos)
    DIG_DDR |= (1 << SEG_G_PIN) | (1 << SEG_DP_PIN) |
               (1 << DIG1_PIN) | (1 << DIG2_PIN) | (1 << DIG3_PIN);

    // Apagar display
    digit_off(1); digit_off(2); digit_off(3);
    write_segments(0, 0);

    uint8_t modo = 0;            // 0 = Tensão, 1 = Corrente
    uint8_t last_btn_state = 1;
    char buffer[80];

    uart_transmit_string("Modo inicial: Tensao");
    uart_new_line();

    while (1) {

        // ============================
        //  LEITURA DO BOTÃO
        // ============================
        uint8_t btn_state = (PIND & (1 << BTN_PIN)) ? 1 : 0;

        if (last_btn_state == 1 && btn_state == 0) {
            // Pressão detetada
            modo = !modo;

            if (modo == 0) uart_transmit_string("Modo alterado para: Tensao");
            else           uart_transmit_string("Modo alterado para: Corrente");

            uart_new_line();
            _delay_ms(200);  // debounce
        }

        last_btn_state = btn_state;

        // ============================================================
        //  MODO TENSÃO
        // ============================================================
        if (modo == 0) {

            adc_set_ref_internal1v1(1);    // usar 1.1V para medir divisor
            _delay_ms(2);

            uint16_t adc_v = adc_read_ch(0);
            float v_adc = adc_to_volts(adc_v, 1);
            float v_in = v_adc * DIVIDER_FACTOR;

            // Envio pela UART
            snprintf(buffer, sizeof(buffer), "V: ADC=%u -> %.2f V", adc_v, v_in);
            uart_transmit_string(buffer);
            uart_new_line();

            // Mostrar no display (1 decimal)
            int scaled10 = (int)(v_in * 10.0f + 0.5f);
            display_show_scaled_int(scaled10);

            // LED ON se tensão > 15V
            if (v_in > 15.0f) DIG_PORT |= (1 << LED_PIN);
            else              DIG_PORT &= ~(1 << LED_PIN);

            adc_set_ref_internal1v1(0); // voltar a 5V para ACS712

        } else {

        // ============================================================
        //  MODO CORRENTE
        // ============================================================

            adc_set_ref_internal1v1(0);   // ACS712 usa ref 5V
            _delay_ms(2);

            uint16_t adc_c = adc_read_ch(0);
            float v_pin = adc_to_volts(adc_c, 0);

            // Fórmula da corrente para ACS712-05B:
            // I = (Vout - 2.5V) / 0.185
            float corrente = (v_pin - ACS712_OFFSET_V) / ACS712_SENSITIVITY;

            // Enviar pela UART
            snprintf(buffer, sizeof(buffer),"I: ADC=%u -> %.3f A", adc_c, corrente);
            uart_transmit_string(buffer);
            uart_new_line();

            // Mostrar no display:
            // Correntes < 1A em mA
            int scaled10;

            if (fabs(corrente) < 1.0f) {
                int mA = (int)(corrente * 1000.0f + 0.5f);
                if (mA < 0) mA = 0;
                if (mA > 999) mA = 999;
                scaled10 = mA * 10;
            } else {
                scaled10 = (int)(corrente * 10.0f + 0.5f);
                if (scaled10 < 0) scaled10 = 0;
                if (scaled10 > 999) scaled10 = 999;
            }

            display_show_scaled_int(scaled10);

            // LED ON se corrente > 1.5 A
            if (fabs(corrente) > 1.5f) DIG_PORT |= (1 << LED_PIN);
            else                       DIG_PORT &= ~(1 << LED_PIN);
        }

        _delay_ms(200);
    }

    return 0;
}
                                                                     