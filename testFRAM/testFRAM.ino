// MAE 155B Test
// FRAM Memory
// Adafruit Part#1895
//
#include <Wire.h>

#define FRAM_ADDRESS 0x50

void write_FRAM(int address, char value)    // memory address range is 0 to 32768
{
  Wire.beginTransmission(FRAM_ADDRESS);
  Wire.write(address >> 8);
  Wire.write(address & 0xFF);
  Wire.write(byte(value));
  Wire.endTransmission();
}

char read_FRAM(int address)
{
  char value;
  
  Wire.beginTransmission(FRAM_ADDRESS);
  Wire.write(address >> 8);
  Wire.write(address & 0xFF);
  Wire.endTransmission();
  Wire.requestFrom(FRAM_ADDRESS, 1); 
  while (Wire.available() < 1);
  value = char(Wire.read());
  
  return value;
}

void setup()
{
  char c_out;
  int test_addr = 256;
  
  Wire.begin();
  Serial.begin(9600);
  
  delay(5000);

  Serial.println("start_test");

  String LOG_message = String("Hello World!");

  Serial.println(LOG_message);

  for (int i=0; i<LOG_message.length(); i++)    // write message to FRAM
  {
    write_FRAM(i+test_addr, LOG_message.charAt(i));
  }

  delay(100);

  for (int i=0; i<LOG_message.length(); i++)    // read message and print
  {
    c_out = read_FRAM(i+test_addr);
    Serial.print(c_out);
  }
  Serial.println();
  
  Serial.println("end_test");
  
}

void loop()
{

}

