#include "arduino.h"

//在16MHz的200ns=>4条指令
void delay_200ns()
{
  __asm__ __volatile__ (
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop");
}

void TIMER_INT()
{
  //我们将覆盖定时器使用步进电机
  //步进电机初始化
  //定时器CTC模式
  // STEPPER MOTORS INITIALIZATION
  // TIMER1 CTC MODE
  TCCR1B &= ~(1 << WGM13);
  TCCR1B |=  (1 << WGM12);
  TCCR1A &= ~(1 << WGM11);
  TCCR1A &= ~(1 << WGM10);
  // output mode = 00 (disconnected)
  TCCR1A &= ~(3 << COM1A0);
  TCCR1A &= ~(3 << COM1B0);
  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
  TCCR1B = (TCCR1B & ~(0x07 << CS10)) | (2 << CS10);

  //OCR1A = 125;  // 16Khz
  //OCR1A = 100;  // 20Khz
  //OCR1A = 80;   // 25Khz
  OCR1A = 160;   // 12.5Khz
  TCNT1 = 0;
  delay(200);
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 interrupt
}

ISR(TIMER1_COMPA_vect)
{
  counter_m[0]++;
  counter_m[1]++;
  if (counter_m[0] >= period_m[0][period_m_index[0]])
  {
    counter_m[0] = 0;
    if (period_m[0][0] == ZERO_SPEED)
      return;
    if (dir_m[0])
      CLR(PORTA, 4);
    //digitalWrite(MOTOR1_DIR, LOW);
    else
      SET(PORTA, 4);
    //digitalWrite(MOTOR1_DIR, HIGH);
    // 我们需要等待，以免200ns的产生步进脉冲...
    period_m_index[0] = (period_m_index[0] + 1) & 0x07; // 周期M指数从0到7
    //    delay_200ns();
    SET(PORTD, 6);
    //digitalWrite(MOTOR1_STEP,HIGH);
    delayMicroseconds(1);
    CLR(PORTD, 6);
    //digitalWrite(MOTOR1_STEP,LOW);
  }
  if (counter_m[1] >= period_m[1][period_m_index[1]])
  {
    counter_m[1] = 0;
    if (period_m[1][0] == ZERO_SPEED)
      return;
    if (dir_m[1])
      CLR(PORTA, 7);
    //digitalWrite(MOTOR2_DIR, LOW);
    else
      SET(PORTA, 7);
    //digitalWrite(MOTOR2_DIR, HIGH);
    period_m_index[1] = (period_m_index[1] + 1) & 0x07;
    //    delay_200ns();
    SET(PORTB, 1);
    //digitalWrite(MOTOR2_STEP,HIGH);
    delayMicroseconds(1);
    CLR(PORTB, 1);
    //digitalWrite(MOTOR2_STEP,LOW);
  }
}
