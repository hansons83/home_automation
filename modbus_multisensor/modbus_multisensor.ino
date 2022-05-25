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

//#define DEBUG_SERIAL_ENABLED
//#define DEBUG_LED_ENABLED

static const int16_t SW_VERSION = 0x0D06;
static const uint8_t EEPROM_VERSION = 0x55;
static const uint8_t MODBUS_DEFAULT_ID = 81;

static const uint8_t INPUT_HIGH_STATE = 0xFF;
static const uint8_t INPUT_LOW_STATE = 0x00;
static const uint8_t INPUTS_NUM = 6;
static const uint8_t INPUTS_INTERRUPT_NUM = 2;
static const uint8_t INPUTS_PINS[INPUTS_NUM] = { A3, A2, 2, 3, 5, 6 };
static uint8_t   inputCounters[INPUTS_NUM] = { 0, 0, 0, 0, 0, 0 };
static int16_t   inputPulses[INPUTS_NUM] = { 0, 0, 0, 0, 0, 0 };
static volatile int16_t inputInterruptPulses[INPUTS_INTERRUPT_NUM] = { 0, 0 };
static uint32_t  inputPulsesTimer = 0;
static uint32_t  PULSES_MEASURE_US = 10000000;
static const uint8_t DS_TEMP_NUM = 4;

static const uint8_t OUTPUTS_NUM = 3;
static const uint8_t OUTPUTS_PINS[OUTPUTS_NUM] = { 13, 12, 11 };

static const uint8_t  CO2_SERIAL_RX_PIN = 10;
static const uint8_t  CO2_SERIAL_TX_PIN = 9;
static const uint32_t CO2_SERIAL_BAUNDRATE = 9600;

#ifdef DEBUG_SERIAL_ENABLED
static const uint8_t  DEBUG_SERIAL_RX_PIN = 12;
static const uint8_t  DEBUG_SERIAL_TX_PIN = 13;
static const uint32_t DEBUG_SERIAL_BAUNDRATE = 9600;
#endif
#ifdef DEBUG_LED_ENABLED
static const uint8_t  DEBUG_LED_PIN = 11;
#endif

//static const uint8_t I2C_SCL_PIN = A5;
//static const uint8_t I2C_SDA_PIN = A5;
static const uint8_t ONEWIRE_PIN = A5;

static const uint32_t RS485_SERIAL_BAUNDRATE = 9600;
static const uint32_t RS485_SERIAL_CONFIG = SERIAL_8N1;
static const uint8_t  RS485_CTRL_PIN = 4;

static const uint8_t ANALOG_INPUTS_NUM = 2;

static const byte LIGHT_SENSOR_PIN = A6;

static const byte SDP810_ADRESS = 0x25;
static const byte SDP811_ADRESS = 0x26;
static const byte SHT31_ADRESS = 0x44;

static uint8_t modbusID = 0;

bool Sdp810Available = false;
bool Sdp811Available = false;
bool BmeAvailable = false;
bool Sht31Available = false;
bool BH1750Available = false;
bool MhzAvailable = false;

struct Registers
{
    int16_t sw_version;   //0

    int16_t temperature;  //1
    int16_t humidity;     //2
    int16_t pressure;     //3
    int16_t co2;          //4
    int16_t lux;          //5
    int16_t lux_prec;     //6
    
    uint16_t inputs_state; //7 97 coil
    
    uint16_t inputs_pulses[INPUTS_NUM]; //8+INPUTS_NUM
    
    uint16_t pulses_time;   //14
    
    uint16_t inputs_value[ANALOG_INPUTS_NUM]; //15+ANALOG_INPUTS_NUM
    
    int16_t SDP1_pressure; //17
    int16_t SDP1_temp;     //18
    int16_t SDP2_pressure; //19
    int16_t SDP2_temp;     //20

    
    uint16_t inputs_counters[INPUTS_NUM]; //21+INPUTS_NUM
    uint16_t outputs_state; // 27
    uint16_t ds_available;  //28
    uint16_t ds_temps[DS_TEMP_NUM]; // 29+DS_TEMP_NUM
    uint16_t interrupt_pulses[INPUTS_INTERRUPT_NUM]; // 33+INPUTS_INTERRUPT_NUM
}
registers;

#define set_bit(var, bit_nr) ((var) |= 1 << (bit_nr))
#define clear_bit(var, bit_nr) ((var) &= ~(1 << (bit_nr)))
#define get_bit(var, bit_nr) (((var) & (1 << (bit_nr))) ? true : false)

void input3InterruptHandler()
{
  static uint32_t lastInt;
  if(millis() > lastInt+5)
  {
    lastInt = millis();
    inputInterruptPulses[0]++;
  }
}
void input4InterruptHandler()
{
  static uint32_t lastInt;
  if(millis() > lastInt+5)
  {
    lastInt = millis();
    inputInterruptPulses[1]++;
  }
}
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
      clear_bit(registers.inputs_state, i);
    }
    else if(inputCounters[i] == INPUT_LOW_STATE)
    {
      set_bit(registers.inputs_state, i);
    }
    if(get_bit(registers.inputs_state, i) != get_bit(lastinputsState, i))
    {
      // Count rising edges
      if(get_bit(registers.inputs_state, i))
      {
        ++inputPulses[i];
        ++registers.inputs_counters[i];
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
    
    noInterrupts();
    registers.interrupt_pulses[0] = inputInterruptPulses[0];
    registers.interrupt_pulses[1] = inputInterruptPulses[1];
    inputInterruptPulses[0] = inputInterruptPulses[1] = 0;
    interrupts();
  }
}

void registerWasRead(uint8_t id)
{
  if(id > 20 && id < 21+INPUTS_NUM)
  {
    ((uint16_t*)&registers)[id] = 0;
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
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire           oneWire(ONEWIRE_PIN);
Modbus            modbusSlave(0, 1, RS485_CTRL_PIN);
NeoSWSerial       mhzSerial(CO2_SERIAL_RX_PIN, CO2_SERIAL_TX_PIN);
#ifdef DEBUG_SERIAL_ENABLED
NeoSWSerial       debugSerial(DEBUG_SERIAL_RX_PIN, DEBUG_SERIAL_TX_PIN);
#endif
BME280I2C         bme;
BH1750            bh1750;
DallasTemperature ds(&oneWire);

#ifdef DEBUG_SERIAL_ENABLED
#define ser_println(a, b, c, d) do{ debugSerial.begin(DEBUG_SERIAL_BAUNDRATE); debugSerial.print(a);debugSerial.print(b);debugSerial.print(c);debugSerial.println(d); debugSerial.end(); }while(0)
#else
#define ser_println(a, b, c, d)
#endif

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

bool spd8xxInit(uint8_t adress)
{
  // continous mode
  uint8_t cmd0[2] = { 0x36, 0x15 };
  byte ret;

  Wire.beginTransmission(adress);
  if (Wire.write(cmd0, 2) != 2) {
      return false;
  }
  if ((ret = Wire.endTransmission(true)) != 0)
  {
    return false;
  }

  return true;
}

bool spd8xxRead(uint8_t adress, int16_t& temp, int16_t& pressure_diff)
{
  const uint8_t DATA_LEN = 9;
  uint8_t data[DATA_LEN] = { 0 };

  Wire.requestFrom(adress, DATA_LEN);
  if (Wire.available() != DATA_LEN) {
      return false;
  }
  for (int i = 0; i < DATA_LEN; ++i) {
      data[i] = Wire.read();
  }

  float raw_pressure = (float)((int16_t)data[0] << 8 | data[1]);
  float raw_scale = (float)((int16_t)data[6] << 8 | data[7]);

  pressure_diff   = (int16_t)((raw_pressure / raw_scale) * 10.0);
  temp = ((int16_t)data[3] << 8 | data[4]) / 20;
  
  ser_println(F("I2C read  "), adress, ", pres ", pressure_diff);
  ser_println(F("I2C read  "), adress, ", temp ", temp);
  return true;
}

bool sht31Init(uint8_t adress)
{
  // continous mode, med rep, 1 mps
  uint8_t cmd0[2] = { 0x20, 0x32 };
  byte ret;

  Wire.beginTransmission(adress);
  if (Wire.write(cmd0, 2) != 2) {
      return false;
  }
  if ((ret = Wire.endTransmission(true)) != 0)
  {
    return false;
  }

  return true;
}
bool sht31Read(uint8_t adress, float& temp, float& hum)
{
  const uint8_t DATA_LEN = 6;
  uint8_t data[DATA_LEN] = { 0 };
  
  uint8_t cmd0[2] = { 0xE0, 0x00 };

  Wire.beginTransmission(adress);
  if (Wire.write(cmd0, 2) != 2) {
      return false;
  }
  if (Wire.endTransmission(true) != 0)
  {
    return false;
  }

  Wire.requestFrom(adress, DATA_LEN);
  if (Wire.available() != DATA_LEN) {
      return false;
  }
  for (int i = 0; i < DATA_LEN; ++i) {
      data[i] = Wire.read();
  }

  uint16_t raw_temp = (uint16_t)data[0] << 8 | (uint16_t)data[1];
  uint16_t raw_hum = (uint16_t)data[3] << 8 | (uint16_t)data[4];

  temp = -45.0 + 175.0 * ((float)raw_temp / (float)0xFFFF);
  hum = 100.0 * ((float)raw_hum / (float)0xFFFF);
  
  ser_println(F("I2C read  "), adress, ", hum ", hum);
  ser_println(F("I2C read  "), adress, ", temp ", temp);
  return true;
}
static const byte EEPROM_VERSION_OFFSET = 0;
static const byte EEPROM_ID_OFFSET = 1;

void scanI2Ctmp()
{
  byte error, address;
  int nDevices;
  ser_println(F("Scanning..."), "", "", "");
  nDevices = 0;
  for(address = 1; address < 255; address++)
  {
  // The i2c_scanner uses the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      ser_println(F("I2C at "), address, "", "");
  
      nDevices++;
    }
  }
}

#define ACS_Pin A3                        //Sensor data pin on A0 analog input

float ACS_Value;                              //Here we keep the raw data valuess
float testFrequency = 50;                    // test signal frequency (Hz)
float windowLength = 40.0/testFrequency;     // how long to average the signal, for statistist
float intercept = 0; // to be adjusted based on calibration testing
float slope = 0.0752; // to be adjusted based on calibration testing
                      //Please check the ACS712 Tutorial video by SurtrTech to see how to get them because it depends on your sensor, or look below
float Amps_TRMS; // estimated actual current in amps

unsigned long printPeriod = 1000; // in milliseconds
// Track time in milliseconds since last reading 
unsigned long previousMillis = 0;

void setup() 
{
  ser_println("Modbus sensor ver: ", SW_VERSION, "", "");
/*
 * 
 */
  memset(&registers, 0, sizeof(Registers));
  registers.sw_version = SW_VERSION;

  if(EEPROM.read(EEPROM_VERSION_OFFSET) != EEPROM_VERSION)
  {
    Serial.println(F("Clearing EEPROM!"));
    EEPROM.write(EEPROM_VERSION_OFFSET, EEPROM_VERSION);
    EEPROM.write(EEPROM_ID_OFFSET, MODBUS_DEFAULT_ID);
  }
  modbusID = EEPROM.read(EEPROM_ID_OFFSET);
/*
 * 
 */
  for(uint8_t i = 0; i < INPUTS_NUM; ++i)
  {
    pinMode(INPUTS_PINS[i], INPUT_PULLUP); // sets the digital pins as input
  }
  attachInterrupt(digitalPinToInterrupt(INPUTS_PINS[2]), input3InterruptHandler, FALLING);
  attachInterrupt(digitalPinToInterrupt(INPUTS_PINS[3]), input4InterruptHandler, FALLING);
  
  pinMode(ACS_Pin,INPUT);  //Define the pin mode
  
#ifndef DEBUG_SERIAL_ENABLED
  for(uint8_t i = 0; i < OUTPUTS_NUM; ++i)
  {
    pinMode(OUTPUTS_PINS[i], OUTPUT);      // sets the digital pins as input
  }
#endif
/*
 * 
 */
  Serial.begin(RS485_SERIAL_BAUNDRATE, RS485_SERIAL_CONFIG);

  modbusSlave.setID(modbusID);
  modbusSlave.begin( 0 );

  // Start up the library
  ds.begin();
  ds.setResolution(10);
  ds.requestTemperatures();
  ds.setWaitForConversion(false);
  registers.ds_available = ds.getDeviceCount();
  if(registers.ds_available > DS_TEMP_NUM)
    registers.ds_available = DS_TEMP_NUM;
  if(!registers.ds_available)
  {
    ser_println("Could not find DS18B20 sensors!", "", "", "");
  }
  else
  {
    ser_println("Found ", registers.ds_available, " DS18B20 sensors", "");
  }
  if(!registers.ds_available)
  {
    Wire.begin();
    //scanI2Ctmp();
    Sdp810Available = spd8xxInit(SDP810_ADRESS);
    if(!Sdp810Available){
      ser_println("Could not find SDP810 sensor!", "", "", "");
    }
    Sdp811Available = spd8xxInit(SDP811_ADRESS);
    if(!Sdp811Available){
      ser_println("Could not find SDP811 sensor!", "", "", "");
    }
    // Start up the library
    BmeAvailable = bme.begin();
    if(!BmeAvailable){
      ser_println("Could not find BME280 sensor!", "", "", "");
    }
    
    // Start up the library
    BH1750Available = bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
    if(!BH1750Available){
      ser_println("Could not find BH1750 sensor!", "", "", "");
    }

    Sht31Available = sht31Init(SHT31_ADRESS);
    if(!Sht31Available){
      ser_println("Could not find SHT31 sensor!", "", "", "");
    }
  }

  mhzSerial.begin(CO2_SERIAL_BAUNDRATE);
  MhzAvailable = true;
  //MhzAvailable = mhz.setRange(MHZ19_RANGE_5000) == MHZ19_RESULT_OK;
  //if(!MhzAvailable){
  //  ser_println("Could not find MH-Z19 sensor!", "", "", "");
  //}
  //mhzSerial.end();
  
  ser_println("Starting ID:", modbusID, "", "");
  
  interrupts();
}

uint32_t lastMillisSensors = ~0;
#ifdef DEBUG_LED_ENABLED
uint32_t lastMillisLed = 0;
#endif
uint32_t lastMillisLight = ~0;
uint32_t lastMillisLightMeasure = ~0;
uint32_t lastMillisAnalog = ~0;
uint32_t lastMillisPrint = ~0;
uint32_t lastMicrosInputs = ~0;

BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
BME280::PresUnit presUnit(BME280::PresUnit_hPa);
float temp(NAN), hum(NAN), pres(NAN);

const float MAX_ADC_READING = 1023.0f;
const float ADC_REF_VOLTAGE = 5.0f;
const float REF_RESISTANCE =  10000.0f;
const float LUX_CALC_SCALAR = 12518931.0f;//6954961.7f;//12518931.0f
const float LUX_CALC_EXPONENT = -1.405;
float lightReadVal = 0.0;
uint16_t lightReadCount = 0;

#define ANEMOMENTER_SPEED_REVOLUTION (2401.0f/2.0f)
void loop()
{
    uint32_t sinceLastCheck;
    sinceLastCheck = calcTimestampDiff(lastMicrosInputs, micros());
    if(sinceLastCheck >= 1000)
    {
      lastMicrosInputs = micros();

      readInputsInterruptHandler(sinceLastCheck);
    }
#ifdef DEBUG_LED_ENABLED
    sinceLastCheck = calcTimestampDiff(lastMillisLed, millis());
    if(sinceLastCheck > 1000)
    {
      lastMillisLed = millis();
      digitalWrite(DEBUG_LED_PIN, digitalRead(DEBUG_LED_PIN) == LOW ? HIGH : LOW);
    }
#endif
    // read analog input
    sinceLastCheck = calcTimestampDiff(lastMillisAnalog, millis());
    if(sinceLastCheck > 2000)
    {
      lastMillisAnalog = millis();
      for(byte i = 0; i < ANALOG_INPUTS_NUM; ++i)
      {
        pinMode(INPUTS_PINS[i], INPUT);

        registers.inputs_value[i] = analogRead(INPUTS_PINS[i]);
        
        pinMode(INPUTS_PINS[i], INPUT_PULLUP);
      }

      if(Sdp810Available)
      {
        spd8xxRead(SDP810_ADRESS, registers.SDP1_temp, registers.SDP1_pressure);
      }
      if(Sdp811Available)
      {
        spd8xxRead(SDP811_ADRESS, registers.SDP2_temp, registers.SDP2_pressure);
      }
    }
    sinceLastCheck = calcTimestampDiff(lastMillisSensors, millis());
    if(sinceLastCheck > 10000)
    {
      lastMillisSensors = millis();

      if(BmeAvailable)
      {
        bme.read(pres, temp, hum, tempUnit, presUnit);
        if(!Sht31Available)
        {
          registers.temperature = (int16_t)(temp * 10.0f);
          registers.humidity = (int16_t)(hum * 10.0f);
        }
        registers.pressure = (int16_t)(pres);
      }
      if(Sht31Available)
      {
        sht31Read(SHT31_ADRESS, temp, hum);
        registers.temperature = (int16_t)(temp * 10.0f);
        registers.humidity = (int16_t)(hum * 10.0f);
      }
      if(registers.ds_available > 0)
      {
        for(uint8_t i = 0; i < registers.ds_available; ++i)
        {
          registers.ds_temps[i] = (int)(ds.getTempCByIndex(i) * 10.0f);
        }
        ds.requestTemperatures();
      }

      if(MhzAvailable)
      {
        mhz19RequestCo2(mhzSerial);
      }
    }
    // Check if response arrived and read it
    mhz19ReadCo2(mhzSerial, registers.co2);


    sinceLastCheck = calcTimestampDiff(lastMillisLightMeasure, millis());
    if(sinceLastCheck > 20)
    {
      lastMillisLightMeasure = millis();
      
      lightReadVal += analogRead(LIGHT_SENSOR_PIN);
      lightReadCount++;
    }
    sinceLastCheck = calcTimestampDiff(lastMillisLight, millis());
    if(sinceLastCheck > 2000)
    {
      lastMillisLight = millis();
      lightReadVal /= lightReadCount;
      //ser_println("Reading analog");
      float resVoltage = lightReadVal * ADC_REF_VOLTAGE / MAX_ADC_READING;
      float ldrResistance = (REF_RESISTANCE * (ADC_REF_VOLTAGE - resVoltage)) / resVoltage;
      
      lightReadVal = 0.0f;
      lightReadCount = 0;
      
      registers.lux = (int16_t)(LUX_CALC_SCALAR * pow(ldrResistance, LUX_CALC_EXPONENT) * 10.0f);
      
      if(BH1750Available)
      {
        registers.lux_prec = (int)(bh1750.readLightLevel() * 10.0f);
      }
      
      ser_println("BH1750: ", registers.lux_prec, ", ", registers.lux);
    }
    /*sinceLastCheck = calcTimestampDiff(lastMillisPrint, millis());
    if(sinceLastCheck > 10000)
    {
      lastMillisPrint = millis();
      
      ser_println("Temp: ", registers.temperature/10.0f, ", hum: ", registers.humidity/10.0f);
      ser_println("Pres: ", registers.pressure, ", light: ", registers.lux);
      ser_println("CO2: ", registers.co2, "Inputs: ", registers.inputs_state);
    }*/
    int res = modbusSlave.poll( (uint16_t*)&registers, sizeof(Registers)/sizeof(int16_t), registerWasRead );
    if(res != 0)
    {
      ser_println("Response send: ", res, "", "");
    }
#ifndef DEBUG_SERIAL_ENABLED
    for(uint8_t i = 0; i < OUTPUTS_NUM; ++i)
    {
      digitalWrite(OUTPUTS_PINS[i], ((registers.outputs_state & (1<<i)) != 0 ? HIGH : LOW));      // sets the digital pins as input
    }
#endif
}

