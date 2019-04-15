#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <arduino.h>

#define BYTE_ENVIADO 0b10000000
#define QTD_BYTES  12
unsigned char dataSource[3][QTD_BYTES];

volatile uint8_t ledVerde = 61;
volatile uint8_t ledVermelho = 61;
volatile unsigned char BYTE_RECEBIDO[QTD_BYTES];
volatile uint16_t valorAD = 0;
volatile uint8_t z = 0;
  
int main(void) {

/*###############################DEFINIÇÕES#########################################*/
  
  Serial.begin(9600);

//Entradas e saídas
  //Entrada para o botão
  DDRB &= 0b11111110;  //Pino PB0 como entrada (push-button)
  //Configução do SPI
  DDRB |= 0b00101100;  //Configurando os pinos MOSI, SCK e SS como saída
  DDRB &= 0b11101111;  //Configurando o pino MISO com entrada
  //Leds
  DDRD |= 0b00111100;  //Pinos PD2, PD3, PD4 e PD5(reset) como saída
  //Botão deve ser definido como pull-up
  PORTB |= 0b00000001; //Pino PB0 com pull-up
  //Conofiguração de redundância: representa que o mestre (uC) está se comunicando com aquele escravo (RFID) em específico. Mas só há um escravo.
  PORTB &= 0b11111011;// Colocando o SS pra 0

  

  //bit SPE habilita o SPI
  //bit MSTR = 1 habilita modo mestre
  //bits SPI2X, SPR1 e SPR0 configuram o clock
  SPCR |= 0b01110000;
  SPSR |= 0b00000001;

  //Valores iniciais dos sinais PWM
  //Configuracao do PWM
  //FAST PWM 10 bits (Modo 7) sem inversão
  TCCR1A = _BV(COM1A1) | _BV(WGM10) | _BV(WGM11);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  ADMUX   |= _BV(REFS0); //AVCC with external capacitor at AREF pin
  ADCSRA  |= _BV(ADEN) | 0b00000111; //Habilitando o ADC(ADEN) e seta o prescale para 128(três ultimos bits)

  ADMUX &= 0b11110000;  //Seta o AD0 como canal de recebimento do sinal analógico

  //CONTAR TEMPO COM O CONTADOR ZERO
  TCCR0A &= 0b00000000;
  TCCR0B |= _BV(CS02) | _BV(CS00); //Clk_{I/O}/1024 (From prescaler)
  TIMSK0 |= _BV(TOIE0);           //Timer/Counter0 Overflow interrupt is enabled
  sei();                          //Habilita a interrupção

  while (1) {
    PORTD &= 0b11011111;
    z = 0;
    while (z < QTD_BYTES + 1) {
      //Transmite dados para o escravo
      //Nível PC0 vai para baixo
      if (z < QTD_BYTES) SPDR = BYTE_ENVIADO;
      else SPDR = 0b00000000;

      while (!(SPSR & (0b10000000)));

      Serial.print("\n");
      Serial.print(SPDR, BIN);
      if (z > 0)BYTE_RECEBIDO[z - 1] = SPDR;
      z++;
    }
    PORTD |= 0b00100000;
    if (!ehNulo(BYTE_RECEBIDO)) {
      if (ehCadastrado(BYTE_RECEBIDO))acessoLiberado();
      else acessoNegado();
    }
    if (!(PINB & 0b00000001)) acessoLiberado();

    ADCSRA |= _BV(ADSC);
    while (!(ADCSRA & 0b00010000));
    valorAD = ADC;

    //Serial.print(valorAD);
    //Serial.print("\n");

    //se o valor lido for maior que 500, liga o led
    if (valorAD/1023.0 <= 0.80) PORTD |= 0b00001000;
    // senão, apaga o led
    else PORTD &= 0b11110111;
  }

}

bool ehCadastrado(unsigned char *tag) {
  //return false;
  bool ehIgual = false;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < QTD_BYTES; j++) {
      if (tag[j] != dataSource[i][j]) {
        ehIgual = false;
        break;
      }
      else ehIgual = true;
    }
    if (ehIgual)break;
  }
  return ehIgual;
}


// Verifica se o valor recebido é nulo, significando a ausência de cartão ou má leitura.
bool ehNulo(unsigned char* tag) {
  for (int i = 8; i < QTD_BYTES; i++) {
    if (tag[i] != (unsigned char)128 && tag[i] != (unsigned char)0) return false;
  }
  return true;
}


void acessoLiberado() { ledVerde = 0; }  //O contador do led verde é zerado para então ficar 1 segundo acesso.
void acessoNegado() { ledVermelho = 0; } //O contador do led vermelho é zerado para então ficar 1 segundo acesso.

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
