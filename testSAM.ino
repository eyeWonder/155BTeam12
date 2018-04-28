// MAE 155B Flight Test
//
// Rx THRO input to D11
// Rx ELEV input to D10
// Rx GEAR input to D9
//
// flap servo to D6
// ESC throttle to D5

#include <Wire.h>
#include <Servo.h>

#define FRAM_ADDRESS 0x50

#define MAG_ADDRESS 0x1E
#define REG_MAG_OUT 0x28

#define ACC_ADDRESS 0x6B
#define REG_ACC_OUT 0x28

#define GYR_ADDRESS 0x6B
#define REG_GYR_OUT 0x22

volatile unsigned long pwm_ch1 = 0;  // D11
volatile unsigned long pwm_ch2 = 0;  // D10
volatile unsigned long pwm_ch3 = 0;  // D9

int ax, ay, az;  // accelerometer (+/-)[16 g]
int mx, my, mz;  // magnetometer (+/-)[4 gauss]
int rx, ry, rz;  // rate gyro (+/-)[2000 deg/s]

int alt;        // range sensor altitude

int addr_count = 0;    // start address for FRAM
int start_msg = 0x1023;  // start message

unsigned long start_time;
int current_time;

Servo flap;
Servo throttle;

void setup()
{
  Wire.begin();
   
  pinMode(9, INPUT);        // GEAR input pin D9
  pinMode(10, INPUT);       // ELEV input pin D10
  pinMode(11, INPUT);       // THRO input D11

  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  
  flap.attach(6, 1000, 2000);  // flap servo D6
  throttle.attach(5, 1000, 2000);  // throttle ESC D5

  flap.writeMicroseconds(1500);
  throttle.writeMicroseconds(1100);
  
  PCMSK0 |= bit(PCINT1);    // interrupt D9
  PCMSK0 |= bit(PCINT2);    // interrupt D10
  PCMSK0 |= bit(PCINT3);    // interrupt D11
  PCIFR |= bit(PCIF0);      // clear interrupts
  PCICR |= bit(PCIE0);      // enable pin change interrupts
  
  init_IMU();

  for (int i=0; i<50; i++)    // flash LED for 20 seconds before start
  {
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(200);
  }

  start_time = millis();

}

void loop()
{

  throttle.writeMicroseconds(pwm_ch1);

  flap.writeMicroseconds(pwm_ch2);
  
  read_ACC();   // accelerometer
  
  read_MAG();   // magnetometer
  
  read_GYR();   // gyro

  alt = analogRead(A0);   // range sensor

  if (addr_count < 32000 && pwm_ch3 < 1300)   // flight mode 2
  {
    digitalWrite(13, HIGH);

    current_time = millis() - start_time;
    if (current_time < 0)
      current_time += 32767;
    
    write_INT(addr_count, start_msg);
    addr_count += 2;
    
    write_INT(addr_count, current_time);
    addr_count += 2;
    
    write_INT(addr_count, ax);
    addr_count += 2;
    write_INT(addr_count, ay);
    addr_count += 2;
    write_INT(addr_count, az);
    addr_count += 2;
    
    write_INT(addr_count, mx);
    addr_count += 2;
    write_INT(addr_count, my);
    addr_count += 2;
    write_INT(addr_count, mz);
    addr_count += 2;
    
    write_INT(addr_count, rx);
    addr_count += 2;
    write_INT(addr_count, ry);
    addr_count += 2;
    write_INT(addr_count, rz);
    addr_count += 2;

    write_INT(addr_count, alt);
    addr_count += 2;

    write_INT(addr_count, int(pwm_ch1));  // throttle
    addr_count += 2;
    write_INT(addr_count, int(pwm_ch2));  // flap
    addr_count += 2;
    
  } else {
     digitalWrite(13, LOW);
  }

}

// FRAM data logging

void write_INT(int address, int value)
{
  byte bb;
  int vv = value;

  bb = byte(vv);
  write_FRAM(address, bb);
  bb = byte(vv >> 8);
  write_FRAM(address+1, bb);
}

void write_FRAM(int address, byte value)    // memory address range is 0 to 32768
{
  Wire.beginTransmission(FRAM_ADDRESS);
  Wire.write(address >> 8);
  Wire.write(address & 0xFF);
  Wire.write(value);
  Wire.endTransmission();
}

// IMU code section

void init_IMU()
{
  Wire.beginTransmission(MAG_ADDRESS);  // enable magnetometer
  Wire.write(0x20);
  Wire.write(0x34);    // 00110100b = med performance, 20 Hz update
  Wire.endTransmission();
  
  Wire.beginTransmission(MAG_ADDRESS); 
  Wire.write(0x22);
  Wire.write(0x00);    // 00000000b = continuous conversion
  Wire.endTransmission();

  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write(0x23);
  Wire.write(0x04);    // 00000100b = z-axis med performance
  Wire.endTransmission();
  
  Wire.beginTransmission(ACC_ADDRESS);  // enable accelerometer
  Wire.write(0x10);
  Wire.write(0x44);  // 01000100b = 100Hz update, 16 g
  Wire.endTransmission();

  Wire.beginTransmission(ACC_ADDRESS);
  Wire.write(0x15);
  Wire.write(0x10);  // 00010000b = normal mode
  Wire.endTransmission();
  
  Wire.beginTransmission(GYR_ADDRESS);  // enable gyro
  Wire.write(0x11);
  Wire.write(0x4C);    // 01001100b = 100Hz update, 2000 deg/s
  Wire.endTransmission();

  Wire.beginTransmission(GYR_ADDRESS);
  Wire.write(0x16);
  Wire.write(0x80);    // 10000000b = normal mode
  Wire.endTransmission();

}

void read_ACC()
{
  uint8_t xlo, xhi;
  uint8_t ylo, yhi;
  uint8_t zlo, zhi;
  
  Wire.beginTransmission((byte) ACC_ADDRESS);
  Wire.write(REG_ACC_OUT);
  Wire.endTransmission();
  Wire.requestFrom((byte) ACC_ADDRESS, (byte) 6);

  while (Wire.available() < 6);
  xlo = Wire.read();
  xhi = Wire.read();
  ylo = Wire.read();
  yhi = Wire.read();
  zlo = Wire.read();
  zhi = Wire.read();

  ax = (xhi << 8 | xlo);
  ay = (yhi << 8 | ylo);
  az = (zhi << 8 | zlo);
}

void read_GYR()
{
  uint8_t xlo, xhi;
  uint8_t ylo, yhi;
  uint8_t zlo, zhi;
  
  Wire.beginTransmission((byte) GYR_ADDRESS);
  Wire.write(REG_GYR_OUT);
  Wire.endTransmission();
  Wire.requestFrom((byte) GYR_ADDRESS, (byte) 6);
  
  while (Wire.available() < 6);
  xlo = Wire.read();
  xhi = Wire.read();
  ylo = Wire.read();
  yhi = Wire.read();
  zlo = Wire.read();
  zhi = Wire.read();
  
  rx = (xhi << 8 | xlo);
  ry = (yhi << 8 | ylo);
  rz = (zhi << 8 | zlo);
}

void read_MAG()
{
  uint8_t xlo, xhi;
  uint8_t ylo, yhi;
  uint8_t zlo, zhi;
  
  Wire.beginTransmission((byte) MAG_ADDRESS);
  Wire.write(REG_MAG_OUT | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom((byte) MAG_ADDRESS, (byte) 6);
  
  while (Wire.available() < 6);
  xlo = Wire.read();
  xhi = Wire.read();
  ylo = Wire.read();
  yhi = Wire.read();
  zlo = Wire.read();
  zhi = Wire.read();
  
  mx = (xhi << 8 | xlo);
  my = (yhi << 8 | ylo);
  mz = (zhi << 8 | zlo);
}

// receiver interrupts

volatile int pwm_state_A = 0;
volatile unsigned long timeA1 = 0, timeA2 = 0, timeA3 = 0;

volatile int pwm_state_B = 0;
volatile unsigned long timeB1 = 0, timeB2 = 0, timeB3 = 0;

volatile int pwm_state_C = 0;
volatile unsigned long timeC1 = 0, timeC2 = 0, timeC3 = 0;

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
  
  if (pwm_state_C != digitalRead(9))
  {
    timeC1 = timeC2;
    timeC2 = timeC3;
    timeC3 = micros();
  
    pwm_ch3 = timeC3 - timeC2;
    if (pwm_ch3 > timeC2 - timeC1) pwm_ch3 = timeC2 - timeC1;
    
    pwm_state_C = !pwm_state_C;
  }
}
