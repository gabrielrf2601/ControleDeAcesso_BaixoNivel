#include <avr/io.h>
#include <math.h>
#include <avr/interrupt.h>
#include <inttypes.h>

#define BYTE_ENVIADO 0b10000000

char dataSource[3][8];

volatile uint8_t ledVerde = 61;
volatile uint8_t ledVermelho = 61;

ISR(TIMER0_OVF_vect) {
  if (ledVerde < 61) {
    PORTD |= 0b00010000;
    ledVerde++;
  } else
    PORTD &= 0b11101111;
  if (ledVermelho < 61) {
    PORTD |= 0b00000100;
    ledVermelho++;
  } else
    PORTD &= 0b11111011;
}


int main(void) {  
  
  char BYTE_RECEBIDO[8];
  uint16_t valorAD = 0;
  uint8_t z = 0;

  DDRB &= 0b11111110;  //Pino PB0 como entrada (push-button)
  PORTB |= 0b00000001; //Pino PB0 com pull-up

  DDRB |= 0b00101100;  //Confugurando os pinos MOSI, SCK e SS como saída
  DDRB &= 0b11101111;  //Confugurando o pino MISO com entrada
  PORTB &= 0b11111011;// colocando o SS pra 0

  DDRD |= 0b00011100;  //Pinos PD0, PD1 e PD2 como saída (leds)

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

  ADMUX   |= _BV(REFS0); //AVCC with external capacitor at AREF pin
  ADCSRA  |= _BV(ADEN) | 0b00000111; //Habilitando o ADC(ADEN) e seta o prescale para 128(três ultimos bits)

  //CONTAR TEMPO COM O CONTADOR ZERO
  TCCR0A &= 0b00000000;
  TCCR0B |= _BV(CS02) | _BV(CS00); //Clk_{I/O}/1024 (From prescaler)
  TIMSK0 |= _BV(TOIE0);           //Timer/Counter0 Overflow interrupt is enabled
  sei();                          //Habilita a interrupção

  while (1) {

    z=0;
    while (z < 9) {
      //Transmite dados para o escravo
      //Nível PC0 vai para baixo
      if(z<8) SPDR = BYTE_ENVIADO;
      else SPDR = 0b00000000;

      while (!(SPSR & (0b10000000)));

      if(z>0)BYTE_RECEBIDO[z-1] = SPDR;
      z++;
      Serial.print(BYTE_RECEBIDO[z]);
    }

    if(ehCadastrado(BYTE_RECEBIDO))acessoLiberado();
    else acessoNegado();
    Serial.print(PINB);
    if(!(PINB & 0b00000001)) acessoLiberado();

    // ADCSRA |= _BV(ADSC);
    // while (!(ADCSRA & 0b00010000));
    // valorAD = ADC;
  }

}

bool ehCadastrado(char *tag){
  bool ehIgual = false;
  for(int i=0; i<3; i++){
    for(int j=0; j<8; j++){
      if(tag[j] != dataSource[i][j]){
        ehIgual = false;
        break;  
      }
      else ehIgual = true;
    }
    if(ehIgual)break;
  }
  return ehIgual;
}

void acessoLiberado() {
  ledVerde = 0;
}

void acessoNegado() {
  ledVermelho = 0;
}
