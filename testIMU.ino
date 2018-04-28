// MAE 155B Test
// Inertial Measurement Unit
// Pololu AltIMU-10 v5
//
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

void setup()
{
  Serial.begin(9600);
  
  Wire.begin();
  
  init_IMU();
}

void loop()
{
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
  
  delay(1000);
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

