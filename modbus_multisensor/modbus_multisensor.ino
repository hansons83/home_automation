/*
 * 
 */
 
#include <ModbusRtu.h>
#include <EEPROM.h>
#include <Wire.h>
#include <OneWire.h>
#include <MsTimer2.h>
#include <NeoSWSerial.h>
#include <BME280.h>
#include <BME280I2C.h>
#include <EnvironmentCalculations.h>
#include <BH1750.h>
#include <DallasTemperature.h>

//#define DEBUG_SERIAL

static const int16_t SW_VERSION = 0x0901;
static const uint8_t EEPROM_VERSION = 0x55;
static const uint8_t MODBUS_DEFAULT_ID = 64;
static const uint8_t MODBUS_CLIENT_IND = 4;

static const uint8_t INPUT_HIGH_STATE = 0xFF;
static const uint8_t INPUT_LOW_STATE = 0x00;
static const uint8_t INPUTS_NUM = 6;
static const uint8_t INPUTS_PINS[INPUTS_NUM] = { A3, A2, 2, 3, 5, 6 };
static uint8_t   inputCounters[INPUTS_NUM] = { 0, 0, 0, 0, 0, 0 };
static int16_t   inputPulses[INPUTS_NUM] = { 0, 0, 0, 0, 0, 0 };
static uint32_t  inputPulsesTimer = 0;
static uint32_t  PULSES_MEASURE_US = 10000000;

static const uint8_t  CO2_SERIAL_RX_PIN = 10;
static const uint8_t  CO2_SERIAL_TX_PIN = 9;
static const uint32_t CO2_SERIAL_BAUNDRATE = 9600;

static const uint8_t  DEBUG_SERIAL_RX_PIN = 12;
static const uint8_t  DEBUG_SERIAL_TX_PIN = 13;
static const uint32_t DEBUG_SERIAL_BAUNDRATE = 9600;
static const uint8_t  DEBUG_LED_PIN = 11;

static const uint8_t I2C_SCL_PIN = A5;
static const uint8_t I2C_SDA_PIN = A5;
static const uint8_t ONEWIRE_PIN = A5;

static const uint32_t RS485_SERIAL_BAUNDRATE = 9600;
static const uint32_t RS485_SERIAL_CONFIG = SERIAL_8N1;
static const uint8_t  RS485_CTRL_PIN = 4;

static const uint8_t ANALOG_INPUTS_NUM = 2;
static const uint8_t ANALOG_INPUTS[] = { A3, A2 };

static const byte LIGHT_SENSOR_PIN = A6;

bool BmeAvailable = false;
bool BH1750Available = false;
bool DsAvailable = false;
bool MhzAvailable = false;

struct Registers
{
    int16_t sw_version;   //0

    int16_t temperature;  //1
    int16_t humidity;     //2
    int16_t pressure;     //3
    int16_t co2;          //4
    int16_t lux;          //5
    
    int16_t inputs_state; //6
    
    int16_t inputs_pulses[INPUTS_NUM]; //7+INPUTS_NUM
    //int16_t in1_pulses;   //7
    //int16_t in2_pulses;   //8
    //int16_t in3_pulses;   //9
    //int16_t in4_pulses;   //10
    //int16_t in5_pulses;   //11
    //int16_t in6_pulses;   //12
    
    uint32_t pulses_time;   //13
    
    int16_t in1_value;    //18
    int16_t in2_value;    //19
    int16_t in9_value;    //20
    int16_t in10_value;   //21
    
    int16_t oneWireTemp1; //22
    int16_t oneWireTemp2; //23
    int16_t oneWireTemp3; //24
    int16_t oneWireTemp4; //25
}
registers;

#define set_bit(var, bit_nr) ((var) |= 1 << (bit_nr))
#define clear_bit(var, bit_nr) ((var) &= ~(1 << (bit_nr)))
#define get_bit(var, bit_nr) (((var) & (1 << (bit_nr))) ? true : false)

void readInputsInterruptHandler(uint32_t elapsedus)
{
  int16_t lastinputsState = registers.inputs_state;
  
  for(uint8_t i = 0; i < INPUTS_NUM; ++i)
  {
    inputCounters[i] <<= 1;
    if(digitalRead(INPUTS_PINS[i]) == LOW)
    {
      inputCounters[i] += 1;
    }
    else
    {
      inputCounters[i] += 0;
    }
    if(inputCounters[i] == INPUT_HIGH_STATE)
    {
      set_bit(registers.inputs_state, i);
    }
    else if(inputCounters[i] == INPUT_LOW_STATE)
    {
      clear_bit(registers.inputs_state, i);
    }
    if(get_bit(registers.inputs_state, i) != get_bit(lastinputsState, i))
    {
      // Count rising edges
      if(get_bit(registers.inputs_state, i))
      {
        ++inputPulses[i];
      }
    }
  }
  inputPulsesTimer += elapsedus;
  if(inputPulsesTimer >= PULSES_MEASURE_US)
  {
    registers.pulses_time = inputPulsesTimer / 1000;
    inputPulsesTimer = 0;
    for(uint8_t i = 0; i < INPUTS_NUM; ++i)
    {
      registers.inputs_pulses[i] = inputPulses[i];
      inputPulses[i] = 0;
    }
  }
}

uint32_t calcTimestampDiff(uint32_t s, uint32_t e)
{
  if(s > e)
  {
    return (0xFFFFFFFFu - (s - e));
  }
  else
  {
    return e - s;
  }
}

/**
 *  Modbus object declaration
 */
// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 8
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
//OneWire           oneWire(ONE_WIRE_BUS);
Modbus            modbusSlave(0, 1, RS485_CTRL_PIN);
NeoSWSerial       mhzSerial(CO2_SERIAL_RX_PIN, CO2_SERIAL_TX_PIN);
#ifdef DEBUG_SERIAL
NeoSWSerial       debugSerial(DEBUG_SERIAL_RX_PIN, DEBUG_SERIAL_TX_PIN);
#endif
BME280I2C         bme;
//BH1750            bh1750;
//DallasTemperature ds(&oneWire);
#ifdef DEBUG_SERIAL
#define ser_println(a, b, c, d) do{ debugSerial.begin(DEBUG_SERIAL_BAUNDRATE); debugSerial.print(a);debugSerial.print(b);debugSerial.print(c);debugSerial.println(d); debugSerial.end(); }while(0)
#else
#define ser_println(a, b, c, d)
#endif
#define ANALOG_PIN       A0
#define INTERRUPT_PIN    7
#define INTERRUPT_ACTIVE HIGH
#define INTERRUPT_MES    5

uint32_t getAvgVal(uint32_t* valArray, byte sIndex, byte count)
{
  uint64_t val = 0;
  
  for(byte i = 0; i < count; ++i)
  {
    val += valArray[(i+sIndex) % count];
  }
  return (uint32_t)(val / count);
}
byte mhz19CalcCRC(byte * response)
{
  byte i;
  byte crc = 0;
  for (i = 1; i < 8; i++)
  {
    crc += response[i];
  }
  crc++;

  return crc;
}

void mhz19RequestCo2(Stream& serial)
{
  byte cmd[9] = { 0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79 };
  //while (serial.available()) { serial.read(); }
  serial.write(cmd, 9);
  serial.flush();
}
bool mhz19ReadCo2(Stream& serial, int16_t& val)
{
  byte response[9];
  int avail = serial.available();
  if(avail >= 9)
  {
    response[0] = serial.read();
    if (response[0] != 0xFF)
    {
      return false;
    }
    
    byte crc = 0;
    for(byte i = 1; i < 8; ++i)
    {
      response[i] = serial.read();
      crc += response[i];
    }
    response[8] = serial.read();
    
    crc = (255 - crc);
    crc++;
    if(crc != response[8])
    {
      return false;
    }

    val = ((int16_t)response[2]) << 8 | (int16_t)response[3];
    return true;
  }
  return false;
}
  
static const byte EEPROM_VERSION_OFFSET = 0;
static const byte EEPROM_ID_OFFSET = 1;

void setup() 
{
  ser_println("Modbus sensor ver: ", SW_VERSION, "", "");
  // Must be changed during firs flashing
  byte modbusID = MODBUS_DEFAULT_ID + 1;
/*
 * 
 */
  memset(&registers, 0, sizeof(Registers));
  registers.sw_version = SW_VERSION;

  if(EEPROM.read(EEPROM_VERSION_OFFSET) != EEPROM_VERSION)
  {
    Serial.println(F("Clearing EEPROM!"));
    EEPROM.write(EEPROM_VERSION_OFFSET, EEPROM_VERSION);
    EEPROM.write(EEPROM_ID_OFFSET, MODBUS_DEFAULT_ID + MODBUS_CLIENT_IND);
  }
  modbusID = EEPROM.read(EEPROM_ID_OFFSET);
/*
 * 
 */
  pinMode(DEBUG_LED_PIN, OUTPUT);
  for(uint8_t i = 0; i < INPUTS_NUM; ++i)
  {
    pinMode(INPUTS_PINS[i], INPUT_PULLUP);      // sets the digital pins as input
  }
/*
 * 
 */
  Serial.begin(RS485_SERIAL_BAUNDRATE, RS485_SERIAL_CONFIG);

  modbusSlave.setID(modbusID);
  modbusSlave.begin( 0 );
  Wire.begin();

/*
  // Start up the library
  ds.begin();
  ds.setResolution(10);
  ds.requestTemperatures();
  ds.setWaitForConversion(false);
  DsAvailable = ds.getDeviceCount() > 0;
  if(!DsAvailable)
  {
    ser_println("Could not find DS18B20 sensor!");
  }
  */
  // Start up the library
  BmeAvailable = bme.begin();
  if(!BmeAvailable){
    ser_println("Could not find BME280 sensor!", "", "", "");
  }
  /*
  // Start up the library
  BH1750Available = bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  if(!BH1750Available){
    ser_println("Could not find BH1750 sensor!");
  }
  */
  mhzSerial.begin(CO2_SERIAL_BAUNDRATE);
  MhzAvailable = true;
  //MhzAvailable = mhz.setRange(MHZ19_RANGE_5000) == MHZ19_RESULT_OK;
  //if(!MhzAvailable){
  //  ser_println("Could not find MH-Z19 sensor!", "", "", "");
  //}
  //mhzSerial.end();
  
  ser_println("Starting ID:", modbusID, "", "");

  //MsTimer2::set(2, readInputsInterruptHandler);
  //MsTimer2::start();
}

uint32_t lastMillis = 0;
uint32_t lastMillisLed = 0;
uint32_t lastMillisLight = 0;
uint32_t lastMillisPrint = 0;
uint32_t lastMicrosInputs = 0;

BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
BME280::PresUnit presUnit(BME280::PresUnit_hPa);
float temp(NAN), hum(NAN), pres(NAN);

#define MAX_ADC_READING 1023.0f
#define ADC_REF_VOLTAGE 5.0f
#define REF_RESISTANCE  10000.0f
#define LUX_CALC_SCALAR 6954961.7f//12518931.0f
#define LUX_CALC_EXPONENT -1.405

#define ANEMOMENTER_SPEED_REVOLUTION (2401.0f/2.0f)
void loop()
{
    uint32_t sinceLastCheck;
    sinceLastCheck = calcTimestampDiff(lastMicrosInputs, micros());
    if(sinceLastCheck >= 2000)
    {
      lastMicrosInputs = micros();

      readInputsInterruptHandler(sinceLastCheck);
    }
    sinceLastCheck = calcTimestampDiff(lastMillisLed, millis());
    if(sinceLastCheck > 1000)
    {
      lastMillisLed = millis();

      digitalWrite(DEBUG_LED_PIN, digitalRead(DEBUG_LED_PIN) == LOW ? HIGH : LOW);
    }
    // read analog input
    sinceLastCheck = calcTimestampDiff(lastMillis, millis());
    if(sinceLastCheck > 10000)
    {
      lastMillis = millis();

      if(BmeAvailable)
      {
        bme.read(pres, temp, hum, tempUnit, presUnit);
        registers.temperature = (int16_t)((temp-0.9f) * 10.0f);
        registers.humidity = (int16_t)(hum * 10.0f);
        registers.pressure = (int16_t)(pres);
      }
      /*if(BH1750Available)
      {
        regData.flux = bh1750.readLightLevel();
      }
      if(DsAvailable)
      {
        registers.temp = (float)((int)(ds.getTempCByIndex(0) * 10.0f)) / 10.0f;
        ds.requestTemperatures();
      }*/

      if(MhzAvailable)
      {
        mhz19RequestCo2(mhzSerial);
      }
    }
    // Check if response arrived and read it
    mhz19ReadCo2(mhzSerial, registers.co2);
    
    sinceLastCheck = calcTimestampDiff(lastMillisLight, millis());
    if(sinceLastCheck > 2000)
    {
      lastMillisLight = millis();
      //ser_println("Reading analog");
      
      float resVoltage = (float)analogRead(LIGHT_SENSOR_PIN) * (ADC_REF_VOLTAGE / MAX_ADC_READING);
      float ldrResistance = (REF_RESISTANCE * (ADC_REF_VOLTAGE - resVoltage)) / resVoltage;
    
      registers.lux = (int16_t)(LUX_CALC_SCALAR * pow(ldrResistance, LUX_CALC_EXPONENT) * 10.0f);
    }
    /*sinceLastCheck = calcTimestampDiff(lastMillisPrint, millis());
    if(sinceLastCheck > 10000)
    {
      lastMillisPrint = millis();
      
      ser_println("Temp: ", registers.temperature/10.0f, ", hum: ", registers.humidity/10.0f);
      ser_println("Pres: ", registers.pressure, ", light: ", registers.lux);
      ser_println("CO2: ", registers.co2, "Inputs: ", registers.inputs_state);
    }*/
    int res = modbusSlave.poll( (uint16_t*)&registers, sizeof(Registers)/sizeof(int16_t) );
    if(res != 0)
    {
      ser_println("Response send: ", res, "", "");
    }
    /*if(Serial.available())
    {
      do
      {
        ser_print(Serial.read());
        ser_print(":");
      }
      while(Serial.available());
    }*/

}

