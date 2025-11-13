#include <Arduino.h>        // Inclui cabeçalhos base do Arduino (não é essencial aqui, mas garante compatibilidade)
#define F_CPU 16000000UL    // Define a frequência do microcontrolador (16 MHz para o Arduino Uno)
#include <avr/io.h>         // Biblioteca para acesso direto aos registos do AVR
#include <util/delay.h>     // Biblioteca para a função _delay_ms()
#include <stdio.h>          // Biblioteca para funções de formatação (snprintf)
#include <stdint.h>         // Biblioteca para tipos inteiros com tamanho definido

// ---------- UART (Comunicação Serial) ----------

// Inicializa a UART (porta série) com uma taxa de transmissão (baud rate) definida
void uart_init(unsigned int baud) {
    // Calcula o valor do divisor (UBRR) com base na frequência do CPU e baud rate
    unsigned int ubrr = (F_CPU / (16UL * baud)) - 1;
    // Divide o valor calculado pelos registos de 8 bits
    UBRR0H = (unsigned char)(ubrr >> 8);   // Parte alta do divisor
    UBRR0L = (unsigned char)ubrr;          // Parte baixa do divisor
    // Ativa o transmissor UART
    UCSR0B = (1 << TXEN0);
    // Configura 8 bits de dados e 1 bit de stop
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Transmite um único carácter pela UART
void uart_transmit(char data) {
    // Espera até que o buffer de transmissão esteja vazio
    while (!(UCSR0A & (1 << UDRE0)));
    // Coloca o carácter no registo de transmissão
    UDR0 = data;
}

// Transmite uma string (sequência de caracteres) pela UART
void uart_transmit_string(const char *str) {
    while (*str) uart_transmit(*str++);  // Envia cada carácter até ao terminador '\0'
}

// Envia uma nova linha (carriage return + line feed)
void uart_new_line() {
    uart_transmit('\r');  // Retorno de carro
    uart_transmit('\n');  // Nova linha
}

// ---------- ADC (Conversor Analógico-Digital) ----------

// Inicializa o ADC
void adc_init() {
    ADMUX = (1 << REFS0); // Define AVcc (5V) como referência de tensão
    // Ativa o ADC e define o prescaler para 128 (16 MHz / 128 = 125 kHz → dentro do intervalo recomendado)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// Lê o valor analógico de um canal (0–7)
uint16_t adc_read(uint8_t ch) {
    ch &= 0x07;                   // Garante que o canal é válido (0 a 7)
    ADMUX = (ADMUX & 0xF8) | ch;  // Seleciona o canal mantendo os bits de referência
    ADCSRA |= (1 << ADSC);        // Inicia a conversão
    while (ADCSRA & (1 << ADSC)); // Espera até a conversão terminar
    return ADC;                   // Retorna o valor de 10 bits (0–1023)
}

// Converte o valor do ADC (0–1023) para milivolts (0–5000 mV)
uint32_t adc_to_millivolts(uint16_t adc_value) {
    // Usa 5000 mV como referência. Se quiseres ser mais preciso, podes medir a tensão real do Vcc.
    return ((uint32_t)adc_value * 5000UL) / 1023UL;
}

// ---------- Programa principal ----------
int main(void) {
    uart_init(9600);  // Inicializa a UART a 9600 bps
    adc_init();       // Inicializa o ADC

    DDRB |= (1 << PB5); // Define o pino PB5 (LED do Arduino Uno) como saída

    char buffer[64];   // Buffer para armazenar texto antes de enviar pela UART

    while (1) {
        // Lê o valor analógico do pino A0 (canal 0)
        uint16_t value = adc_read(0);

        // Converte o valor lido para milivolts
        uint32_t mv = adc_to_millivolts(value);

        // Divide a tensão em parte inteira (volts) e duas casas decimais (centésimos)
        uint16_t volts = mv / 1000;        // Parte inteira (ex: 2)
        uint16_t centi = (mv % 1000) / 10; // Parte decimal com 2 dígitos (ex: 47 -> 0.47V)

        // Formata o texto a enviar: "ADC: 512 -> 2.50 V"
        snprintf(buffer, sizeof(buffer), "ADC: %u -> %u.%02u V", value, volts, centi);
        uart_transmit_string(buffer);  // Envia a mensagem pela UART
        uart_new_line();               // Nova linha para legibilidade

        // Se a tensão for maior que 2,5V, acende o LED; caso contrário, apaga
        if (mv > 2500) {
            PORTB |= (1 << PB5);   // LED ON
        } else {
            PORTB &= ~(1 << PB5);  // LED OFF
        }

        _delay_ms(500); // Espera 500 ms antes da próxima leitura
    }
}
                                                                         