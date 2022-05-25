/*
 * 
 */
 
#include <ModbusRtu.h>
#include <EEPROM.h>
//#include <Wire.h>
//#include <OneWire.h>
//#include <MsTimer2.h>
#include <NeoSWSerial.h>
//#include <BME280.h>
//#include <BME280I2C.h>
//#include <EnvironmentCalculations.h>
//#include <BH1750.h>
//#include <DallasTemperature.h>
#include <Filters.h>

#define DEBUG_SERIAL_ENABLED
//#define DEBUG_LED_ENABLED
static const int16_t SW_VERSION = 0x0101;
static const uint8_t EEPROM_VERSION = 0x56;
static const uint8_t MODBUS_DEFAULT_ID = 25;

#ifdef DEBUG_SERIAL_ENABLED
static const uint8_t  DEBUG_SERIAL_RX_PIN = 12;
static const uint8_t  DEBUG_SERIAL_TX_PIN = 13;
static const uint32_t DEBUG_SERIAL_BAUNDRATE = 9600;
#endif
#ifdef DEBUG_LED_ENABLED
static const uint8_t  DEBUG_LED_PIN = 11;
#endif

static const uint32_t RS485_SERIAL_BAUNDRATE = 9600;
static const uint32_t RS485_SERIAL_CONFIG = SERIAL_8N1;
static const uint8_t  RS485_CTRL_PIN = 4;

#define ANALOG_INPUTS_NUM 2

static uint8_t modbusID = 0;

struct Registers
{
    int16_t sw_version;   //0
    uint16_t inputs_value[ANALOG_INPUTS_NUM]; //1+ANALOG_INPUTS_NUM
    uint16_t inputs_current[ANALOG_INPUTS_NUM]; //3+ANALOG_INPUTS_NUM
}
registers;

void registerWasRead(uint8_t id)
{
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
Modbus            modbusSlave(0, 1, RS485_CTRL_PIN);
#ifdef DEBUG_SERIAL_ENABLED
NeoSWSerial       debugSerial(DEBUG_SERIAL_RX_PIN, DEBUG_SERIAL_TX_PIN);
#endif

#ifdef DEBUG_SERIAL_ENABLED
#define ser_println(a, b, c, d) do{ debugSerial.print(a);debugSerial.print(b);debugSerial.print(c);debugSerial.println(d); debugSerial.end(); }while(0)
#else
#define ser_println(a, b, c, d)
#endif

static const byte EEPROM_VERSION_OFFSET = 0;
static const byte EEPROM_ID_OFFSET = 1;
uint8_t ACS_Pin[ANALOG_INPUTS_NUM]= { A0, A1 };                        //Sensor data pin on A0 analog input

float testFrequency = 50;                    // test signal frequency (Hz)
float windowLength = 40.0/testFrequency;     // how long to average the signal, for statistist

float Amps_TRMS; // estimated actual current in amps
float ACS_Value; //Here we keep the raw data valuess
float Sigma_Value; //Here we keep the raw data valuess

unsigned long printPeriod = 1000; // in milliseconds
// Track time in milliseconds since last reading 
unsigned long previousMillis = 0;
RunningStatistics inputStats[ANALOG_INPUTS_NUM];                 // create statistics to look at the raw test signal

void setup() 
{
#ifdef DEBUG_SERIAL_ENABLED
  debugSerial.begin(DEBUG_SERIAL_BAUNDRATE); 
#endif
  ser_println("Modbus sensor ver: ", SW_VERSION, "", "");
/*
 * 
 */
  memset(&registers, 0, sizeof(Registers));
  registers.sw_version = SW_VERSION;

  if(EEPROM.read(EEPROM_VERSION_OFFSET) != EEPROM_VERSION)
  {
    ser_println("Clearing EEPROM!", "", "", "");
    EEPROM.write(EEPROM_VERSION_OFFSET, EEPROM_VERSION);
    EEPROM.write(EEPROM_ID_OFFSET, MODBUS_DEFAULT_ID);
  }
  modbusID = EEPROM.read(EEPROM_ID_OFFSET);
/*
 * 
 */
  Serial.begin(RS485_SERIAL_BAUNDRATE, RS485_SERIAL_CONFIG);

  modbusSlave.setID(modbusID);
  modbusSlave.begin( 0 );

  for(int i = 0; i < ANALOG_INPUTS_NUM; ++i)
  {
    pinMode(ACS_Pin[i],INPUT);  //Define the pin mode
    inputStats[i].setWindowSecs( windowLength );     //Set the window length
  }

  ser_println("Starting ID:", modbusID, "", "");
}

#ifdef DEBUG_LED_ENABLED
uint32_t lastMillisLed = 0;
#endif
uint32_t lastMillisAnalog = ~0;

float intercept[ANALOG_INPUTS_NUM] = {-0.07f, -0.07f}; // to be adjusted based on calibration testing
float slope[ANALOG_INPUTS_NUM] = {0.057973556f, 0.058553329f }; // to be adjusted based on calibration testing
                      //Please check the ACS712 Tutorial video by SurtrTech to see how to get them because it depends on your sensor, or look below

void loop()
{
    uint32_t sinceLastCheck;

    for(int i = 0; i < ANALOG_INPUTS_NUM; ++i)
    {
      ACS_Value = analogRead(ACS_Pin[i]);  // read the analog in value:
      inputStats[i].input(ACS_Value);  // log to Stats function
    }
    
    sinceLastCheck = calcTimestampDiff(lastMillisAnalog, millis());
    if(sinceLastCheck > 1000)
    {
      lastMillisAnalog = millis();
 
      for(int i = 0; i < ANALOG_INPUTS_NUM; ++i)
      {
        Sigma_Value = inputStats[i].sigma();
        registers.inputs_value[i] = Sigma_Value * 100;
        Amps_TRMS = (((5.0/1023.0 * Sigma_Value) / 270.0) * 1000.0) - 0.02;
        if(Amps_TRMS < 0.0f)
        {
          Amps_TRMS = 0.0f;
        }
          
        registers.inputs_current[i] = Amps_TRMS * 1000;

        debugSerial.println(registers.inputs_current[i]);
        ser_println("RMS: ", Amps_TRMS, ", sigma: ", registers.inputs_value[i]);
      }
      ser_println("", "", "", "");
    }
    int res = modbusSlave.poll( (uint16_t*)&registers, sizeof(Registers)/sizeof(int16_t), registerWasRead );
    if(res != 0)
    {
      ser_println("Response send: ", res, "", "");
    }
}

