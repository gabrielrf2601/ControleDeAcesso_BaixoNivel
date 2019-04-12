#include <avr/io.h>
#include <math.h>
#include <avr/interrupt.h>
#include <inttypes.h>

#define BYTE_ENVIADO 0b10000000

volatile uint8_t x = 0;
volatile uint8_t z = 0;

ISR(TIMER0_OVF_vect) {
  x++;
  if (x < 61) {
    PORTB |= 0b00100000;
  } else if (x < 122) {
    PORTB &= 0b11011111;
  } else
    x = 0;
}


int main(void) {
  char BYTE_RECEBIDO[8][8];
  
  DDRB &= 0b11111110;  //Pino PB0 como entrada (push-button)
  PORTB |= 0b00000001; //Pino PB0 com pull-up

  DDRB |= 0b00101100;  //Confugurando os pinos MOSI, SCK e SS como saída
  DDRB &= 0b11101111;  //Confugurando o pino MISO com entrada

  DDRD |= 0b00000111;  //Pinos PD0, PD1 e PD2 como saída (leds)

  //bit SPE habilita o SPI
  //bit MSTR = 1 habilita modo mestre
  //bits SPI2X, SPR1 e SPR0 configuram o clock
  SPCR |= 0b01010000;
  SPSR |= 0b00000001;

  //Valores iniciais dos sinais PWM
  //Configuracao do PWM
  //FAST PWM 10 bits (Modo 7) sem inversão
  TCCR1A = _BV(COM1A1) | _BV(WGM10) | _BV(WGM11);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  ADMUX   |= 0b01000000; //AVCC with external capacitor at AREF pin
  ADCSRA  |= 0b10000111; //Habilitando o ADC e seta o prescale para 128

  //CONTAR TEMPO COM O CONTADOR ZERO
  TCCR0A &= 0b00000000;
  TCCR0B |= _BV(CS02) | _BV(CS00); //Clk_{I/O}/1024 (From prescaler)
  TIMSK0 |= _BV(TOIE0);           //Timer/Counter0 Overflow interrupt is enabled
  sei();                          //Habilita a interrupção

  while (1) {
    while (z < 8) {
      //Transmite dados para o escravo
      //Nível PC0 vai para baixo
      PORTB &= 0b11111011;
      SPDR = BYTE_ENVIADO;
      while (!(SPSR & (0b10000000)));
      BYTE_RECEBIDO[z++] = SPDR;
    }

    ADCSRA |= 0b01000000;
    while (!(ADCSRA & 0b00010000)) {
      OCR1A = ADC;
    }
  }

}
