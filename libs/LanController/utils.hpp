#pragma once
#include <Arduino.h>
#include <SoftWire.h>
#include <OneWire.h>
/*
 *
 */
#define set_bit(var, bit_nr) ((var) |= 1 << (bit_nr))
#define clear_bit(var, bit_nr) ((var) &= ~(1 << (bit_nr)))
#define get_bit(var, bit_nr) (((var) & (1 << (bit_nr))) ? true : false)

/*
 *
 */
void scanI2C(SoftWire& sWire)
{
  byte error, address;
  int nDevices;
  Serial.println(F("Scanning..."));
  nDevices = 0;
  for(address = 1; address < 255; address++)
  {
  // The i2c_scanner uses the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.
    sWire.beginTransmission(address);
    error = sWire.endTransmission();

    if (error == 0)
    {
      Serial.print(F("I2C at 0x"));
      if (address<16) 
        Serial.print(F("0"));
      Serial.print(address, HEX);
      Serial.println(F("!"));
  
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print(F("Err at 0x"));
      if (address<16)
        Serial.print(F("0"));
      Serial.println(address,HEX);
    }
  }
}

/*
const char hex_to_char[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
inline void toHexStr (byte b, char* target)
{
  target[0] = hex_to_char[b>>4];
  target[1] = hex_to_char[b&0x0F];
}*/
/*
 *
 */
inline void byteToHexStr (byte b, char* target)
{
  target[0] = (b>>4) > 9 ? (b>>4)-10 + 'A' : (b>>4) + '0';
  target[1] = (b&0x0F) > 9 ? (b&0x0F)-10 + 'A' : (b&0x0F) + '0';
}
/*
 *
 */
uint32_t calcTimestampDiff(uint32_t s, uint32_t e)
{
  if(s > e)
  {
    //ser_println("Overflow");
    return (0xFFFFFFFFu - (s - e));
  }
  else
  {
    return e - s;
  }
}
/*
 *
 */
void MCP27008_setup(SoftWire& wire, byte adress)
{
  uint8_t error;
  wire.beginTransmission(adress);      // start talking to the device
  wire.write(0x00);                   // select the IODIR register
  wire.write(0x00);                   // set register value-all low, sets all pins as outputs on MCP23008
  error = wire.endTransmission();             // stop talking to the devicevice
  if(error != 0)
  {
    Serial.print(F("MCP setup er: "));
    Serial.println(error);
  }
}
/*
 *
 */
uint8_t MCP27008_write(SoftWire& wire, byte adress, uint8_t value)
{
  uint8_t error;
  wire.beginTransmission(adress);
  wire.write(0x09);                   // select the GPIO register
  wire.write(value);                  // set register value
  error = wire.endTransmission();
  if(error != 0)
  {
    Serial.print(F("MCP write er: "));
    Serial.println(error);
  }
  return error;
}
/*
 *
 */
bool getMacAddress(OneWire& oneWire, byte* target)
{
  int8_t i;           // This is for the for loops
  //byte crc_calc;    //calculated CRC
  //byte crc_byte;    //actual CRC as sent by ds24012401
  //1-Wire bus reset, needed to start operation on the bus,
  //returns a 1/TRUE if presence pulse detected
  target[0] = 0x6C;
  target[1] = 0x75;
  if (oneWire.reset() == true)
  {
    oneWire.write(0x33);  //Send Read data command
    //Serial.print("FC: 0x");
    //PrintTwoDigitHex (oneWire.read(), 1);
    //Serial.print("HD: ");
    oneWire.read();
    for (i = 3; i >= 0; i--)
    {
      target[i+2] = oneWire.read();
    }
	return true;
  }
  return false;
}
