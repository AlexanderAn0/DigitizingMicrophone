/**
    Оцифровка сигнала с микрофона с частотой 8кГц, отсчеты 16бит
    отправляет результат оцифровки по последовательному порту на компьютер 
     Arduino Nano; atmega328
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#define FREQ_DIGIT_KHZ 8000    //< частота опроса аналогового сигнала
#define DUR_OF_ONE_PACK 0.005   //< через сколько секунд отправляется пакет данных
#define CONV_IN_ONE_PACK FREQ_DIGIT_KHZ*DUR_OF_ONE_PACK   //< количество измерений в одном пакете данных
#define HEAD_LEN_UINT16 1       //< длинна заголовка пакета данных
#define MK_CLK 16000000         //< тактовая частота процессора
#define TIM_CLK_DIV 8           //< делитель частоты МК для таймера
const uint8_t NUM_CONV_PLUS_HEAD_UINT16 = CONV_IN_ONE_PACK + HEAD_LEN_UINT16;  //< количество 16 бит сэмплов в пакете
const uint8_t NUM_CONV_PLUS_HEAD_CHAR = NUM_CONV_PLUS_HEAD_UINT16 * 2;         //< количество байт в пакете
const uint8_t TIM_SCORE_FOR_8KHZ = MK_CLK / TIM_CLK_DIV / FREQ_DIGIT_KHZ - 1; //< значение регистра совпадения для таймера в режиме CTC
const uint16_t HEADER = 0xFFFF; //< заголовок пакета

uint8_t bytesCount = 1;         //< позиция для записи сэмпла в буфер

uint16_t bufferADC[2][NUM_CONV_PLUS_HEAD_UINT16]; //< буфер для значений с АЦП

bool sendBuffer = false;        //< флаг, разрешающий отправку сообщения
uint8_t curBufIndex = 0;        //< номер буфера заполняемого на данный момент

uint16_t * BufferForFillPtr = bufferADC[curBufIndex]; //< указатель на заполняемый буфер

enum {
  RES_LEFT_ORIENT = 1,
  PIN_ADC3 = 3,

  CLK_DIV_8 = 2,
  TIM_OP_MODE_CTC = 2,
  INTERRUPT_CTC_ON = 2,
  INTERRUPT_OFF = 0,
  RESET = 0,

  CLK_DIV_32 = 5,
  START_FLAG_BY_TIM_INTERR = 3,

  ON = 1,
};

void setup() {

  cli();                      ///< сброс флагов прерывания

  /// таймер 0 по совпадению 
  TCCR0A = RESET;
  TCCR0B = RESET;
  TCCR0A = (TIM_OP_MODE_CTC << WGM00);  ///< прерывание по совпадению
  TCCR0B = (CLK_DIV_8 << CS00);        ///< деление частоты clk/8
  OCR0A = TIM_SCORE_FOR_8KHZ;                ///< при делителе clk/8 частота 8кГц при совпадении с этим числом
  TIMSK0 = (ON << OCIE0A);    ///< вкл прерывания по совпадению


  ADMUX = (ON << REFS0) | (ON << ADLAR) | (PIN_ADC3 << MUX0);
  ADCSRA = (ON << ADEN) | (ON << ADATE) | (ON << ADIE) | (CLK_DIV_32 << ADPS0)  ; ///< вкл АЦП; запуск 1го преобразования; Предделитель частоты: 128
  ADCSRB |= (START_FLAG_BY_TIM_INTERR << ADTS0);
  sei();                      ///< включение глобальных прерываний

  Serial.begin(230400);        ///< включение последовательного порта



  bufferADC[0][0] = HEADER;
  bufferADC[1][0] = HEADER;
  BufferForFillPtr = bufferADC[0];

}

/// прерывания таймера 0 с частотой 8кГц запускает преобразование ацп
ISR(TIMER0_COMPA_vect) {}


/// по завершению преобразования сохраняет в один из буферов значения, и дает сигнал на отправку в
/// последовательный порт, если буфер наполнился, переходит к наполнению другого
ISR(ADC_vect)
{
  BufferForFillPtr[bytesCount++] = ADCL | ((uint16_t)ADCH << 8);
  if (bytesCount >= NUM_CONV_PLUS_HEAD_UINT16)
  {

    bytesCount = 1;
    curBufIndex ^= 1;    // Инверсия бита: получаем чередование 0 или 1
    BufferForFillPtr = bufferADC[curBufIndex];    // Здесь указатель уже будет указывать на другой буфер
    sendBuffer = true;
  }
}

/// при поступлении синала на отправку, отправляет буфер, в который сейчас не пишет АЦП
void loop()
{
  if (sendBuffer)
  { 
    Serial.write((char *)bufferADC[curBufIndex ^ 1], NUM_CONV_PLUS_HEAD_CHAR);

    sendBuffer = false;
  }
}
