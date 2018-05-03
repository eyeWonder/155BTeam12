// MAE 155B Flight Test
// Read FRAM Data
//

#include <Wire.h>

#define FRAM_ADDRESS 0x50

int addr_count = 0;    // start address for FRAM

int time_stamp;
int ax, ay, az;
int mx, my, mz;
int rx, ry, rz;
int pwm_ch1, pwm_ch2;
int alt;

void setup()
{
  byte b1;
  byte b2;
  
  Serial.begin(9600);
  
  Wire.begin();    // open I2C connection with FRAM
  
  delay(5000);  // wait for Serial Monitor

  while (addr_count < 32000)
  {
    b1 = read_FRAM(addr_count);
    b2 = read_FRAM(addr_count+1);
    
    if ((b1 == 0x23) && (b2 == 0x10))
    {
      addr_count += 2;
      
      time_stamp = read_INT(addr_count);
      addr_count += 2;
      
      ax = read_INT(addr_count);
      addr_count += 2;
      ay = read_INT(addr_count);
      addr_count += 2;
      az = read_INT(addr_count);
      addr_count += 2;

      mx = read_INT(addr_count);
      addr_count += 2;
      my = read_INT(addr_count);
      addr_count += 2;
      mz = read_INT(addr_count);
      addr_count += 2;

      rx = read_INT(addr_count);
      addr_count += 2;
      ry = read_INT(addr_count);
      addr_count += 2;
      rz = read_INT(addr_count);
      addr_count += 2;

      alt = read_INT(addr_count);
      addr_count += 2;

      pwm_ch1 = read_INT(addr_count);
      addr_count += 2;
      pwm_ch2 = read_INT(addr_count);
      addr_count += 2;
      
      Serial.print(time_stamp);
      Serial.print(",");
      Serial.print(ax);
      Serial.print(",");
      Serial.print(ay);
      Serial.print(",");
      Serial.print(az);
      Serial.print(",");
      Serial.print(mx);
      Serial.print(",");
      Serial.print(my);
      Serial.print(",");
      Serial.print(mz);
      Serial.print(",");
      Serial.print(rx);
      Serial.print(",");
      Serial.print(ry);
      Serial.print(",");
      Serial.print(rz);
      Serial.print(",");
      Serial.print(alt);
      Serial.print(",");
      Serial.print(pwm_ch1);
      Serial.print(",");
      Serial.println(pwm_ch2);
      
    } else {
      addr_count++;
    }
      
  }
  
}

void loop()
{
 
}

int read_INT(int address)
{
  byte bb;
  int vv = 0;

  bb = read_FRAM(address+1);
  vv = byte(bb);
  bb = read_FRAM(address);
  vv = (vv << 8) | bb;

  return vv;
}

byte read_FRAM(int address)
{
  byte value;
  
  Wire.beginTransmission(FRAM_ADDRESS);
  Wire.write(address >> 8);
  Wire.write(address & 0xFF);
  Wire.endTransmission();
  Wire.requestFrom(FRAM_ADDRESS, 1);
  
  while (Wire.available() < 1);
  value = Wire.read();
  
  return value;
}


