// MAE 155B Test
// Radio Receiver
// Spectrum AR610
//
// Rx THRO input to D11
// Rx ELEV input to D10
// Rx GEAR input to D9
#include <Servo.h>
#include <Wire.h>

#define MAG_ADDRESS 0x1E
#define REG_MAG_OUT 0x28

#define BAR_ADDRESS 0x5D
#define REG_BAR_OUT 0x28

#define ACC_ADDRESS 0x6B
#define REG_ACC_OUT 0x28

#define GYR_ADDRESS 0x6B
#define REG_GYR_OUT 0x22

int ax, ay, az;  // accelerometer (+/-)[16 g]
int mx, my, mz;  // magnetometer (+/-)[4 gauss]
int rx, ry, rz;  // rate gyro (+/-)[2000 deg/s]
long pr_atm;     // atm pressure [260 to 1260 hPa]


Servo myservo;
Servo motor;
volatile unsigned long pwm_ch1 = 0;  // D11
volatile unsigned long pwm_ch2 = 0;  // D10
float throttle = 0;
float servo = 0;
void setup()
{
  
  Serial.begin(115200);
  Wire.begin();
  init_IMU();


  
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

int moveflap(){
  float mx_g = float(mx)*0.146;    // convert magnetometer to milligauss
  float my_g = float(my)*0.146;
  float psi  = acos(mx_g/sqrt(mx_g*mx_g+my_g*my_g));
  Serial.println(psi);
  int flap =  250*acos(psi-0.85)-280;
  Serial.println(flap);
  return flap;
}
void loop()
{  
  printIMU();
  servo = constrain((uint32_t)180*abs(pwm_ch2-1192)/(1800-1192), 0, 180);
  throttle = constrain((uint32_t)45 + 55*abs(pwm_ch1-1072)/(1908-1072), 45, 100);
  if (pwm_ch1>1000) motor.write(throttle);
  myservo.write(moveflap());
  Serial.print(throttle);
  Serial.print(" ");
  Serial.print(pwm_ch1);
  Serial.print(" ");
  Serial.println(pwm_ch2);
  delay(15);
  
}
void printIMU(){
      read_MAG();
  
  float mx_g = float(mx)*0.146;    // convert magnetometer to milligauss
  float my_g = float(my)*0.146;
  float mz_g = float(mz)*0.146;

  Serial.print("magnet: ");
  Serial.print(mx_g);
  Serial.print(" ");
  Serial.print(my_g);
  Serial.print(" ");
  Serial.println(mz_g);
  
  read_ACC();
  
  float ax_g = float(ax)*0.000488;    // convert accelerometer to g's
  float ay_g = float(ay)*0.000488;
  float az_g = float(az)*0.000488;
  
  Serial.print("accel: ");
  Serial.print(ax_g);
  Serial.print(" ");
  Serial.print(ay_g);
  Serial.print(" ");
  Serial.println(az_g);
 
  read_GYR();
  
  float rx_d = float(rx)*0.07;    // convert gyro to deg/s
  float ry_d = float(ry)*0.07;
  float rz_d = float(rz)*0.07;
  
  Serial.print("gyro: ");
  Serial.print(rx_d);
  Serial.print(" ");
  Serial.print(ry_d);
  Serial.print(" ");
  Serial.println(rz_d);
  
  read_BAR();
  
  float press_hPa = float(pr_atm)/4096;  // convert pressure to hPa
  float alt_m = 44332.3*(1.0 - pow(press_hPa/1013.25, 0.190357));  // altitude in meters
  
  Serial.print("baro: ");
  Serial.print(press_hPa);
  Serial.print(" ");
  Serial.println(alt_m);
  Serial.println(" ");
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

  Wire.beginTransmission(BAR_ADDRESS);  // enable barometer
  Wire.write(0x20);
  Wire.write(0xB0);    // 10110000b = 12.5Hz update, continuous 
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

void read_BAR()
{
  uint8_t pxl, pl, ph;
    
  Wire.beginTransmission((byte) BAR_ADDRESS);
  Wire.write(REG_BAR_OUT | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom((byte) BAR_ADDRESS, (byte) 3);
  
  while (Wire.available() < 3);
  pxl = Wire.read();
  pl = Wire.read();
  ph = Wire.read();
  
  pr_atm = (int32_t)(int8_t)ph << 16 | (uint16_t)pl << 8 | pxl;
}

