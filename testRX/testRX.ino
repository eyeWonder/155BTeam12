// MAE 155B Test
// Radio Receiver
// Spectrum AR610
//
// Rx THRO input to D11
// Rx ELEV input to D10
// Rx GEAR input to D9
#include <Servo.h>

Servo myservo;
Servo motor;
volatile unsigned long pwm_ch1 = 0;  // D11
volatile unsigned long pwm_ch2 = 0;  // D10
float throttle = 0;
float servo = 0;
void setup()
{
  
  Serial.begin(9600);
  myservo.attach(6);
  motor.attach(4);
  motor.writeMicroseconds(1000);
  delay(3000);
  pinMode(9, INPUT);        // GEAR input pin D9
  pinMode(10, INPUT);       // ELEV input pin D10
  pinMode(11, INPUT);       // THRO input D11
  PCMSK0 |= bit(PCINT1);    // interrupt D9
  PCMSK0 |= bit(PCINT2);    // interrupt D10
  PCMSK0 |= bit(PCINT3);    // interrupt D11
  PCIFR |= bit(PCIF0);      // clear interrupts
  PCICR |= bit(PCIE0);      // enable pin change interrupts
}

void loop()
{
  
  servo = constrain((uint32_t)180*abs(pwm_ch2-1192)/(1800-1192), 0, 180);
  throttle = constrain((uint32_t)45 + 55*abs(pwm_ch1-1072)/(1908-1072), 45, 100);
  if (pwm_ch1>1000) motor.write(throttle);
  myservo.write(servo);
  Serial.print(throttle);
  Serial.print(" ");
  Serial.print(pwm_ch1);
  Serial.print(" ");
  Serial.println(pwm_ch2);
  delay(15);
  
}

// receiver interrupts

volatile int pwm_state_A = 0;
volatile unsigned long timeA1 = 0, timeA2 = 0, timeA3 = 0;

volatile int pwm_state_B = 0;
volatile unsigned long timeB1 = 0, timeB2 = 0, timeB3 = 0;

// interrupt handler

ISR (PCINT0_vect)
{
  if (pwm_state_A != digitalRead(11))
  {
    timeA1 = timeA2;
    timeA2 = timeA3;
    timeA3 = micros();
  
    pwm_ch1 = timeA3 - timeA2;
    if (pwm_ch1 > timeA2 - timeA1) pwm_ch1 = timeA2 - timeA1;
    
    pwm_state_A = !pwm_state_A;
    analogWrite( 5, pwm_ch1);

    
  }
  
  if (pwm_state_B != digitalRead(10))
  {
    timeB1 = timeB2;
    timeB2 = timeB3;
    timeB3 = micros();
  
    pwm_ch2 = timeB3 - timeB2;
    if (pwm_ch2 > timeB2 - timeB1) pwm_ch2 = timeB2 - timeB1;
    
    pwm_state_B = !pwm_state_B;
  }
}
