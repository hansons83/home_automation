

/*
 * 
 */
#include <ModbusRtu.h>
#include <EEPROM.h>
#include <NeoSWSerial.h>
#include <OneWire.h>

#include "emonLibCM.h"

//#define DEBUG_SERIAL_ENABLED
//#define DEBUG_LED_ENABLED
static const int16_t SW_VERSION = 0x0012;
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

struct Registers
{
    uint16_t sw_version;   //0
    uint16_t voltage;
    uint16_t current[ANALOG_INPUTS_NUM];
    uint16_t power[ANALOG_INPUTS_NUM];
    int32_t  watt_hour[ANALOG_INPUTS_NUM];
    int16_t  frequency;
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
//#define ser_println(a, b, c, d) do{ debugSerial.print(a);debugSerial.print(b);debugSerial.print(c);debugSerial.println(d); }while(0)
#else
#define ser_println(a, b, c, d)
#endif

static const byte EEPROM_VERSION_OFFSET = 0; //1 byte
static const byte EEPROM_ID_OFFSET = 1;      //1 byte
static const byte EEPROM_ENERGY_OFFSET = 2;  //8 bytes

void setup() 
{
#ifdef DEBUG_SERIAL_ENABLED
  debugSerial.begin(DEBUG_SERIAL_BAUNDRATE); 
#endif
  ser_println("Modbus current meter ver: ", SW_VERSION, "", "");
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
    for(byte i = 0; i < ANALOG_INPUTS_NUM; ++i)
    {
      EEPROM.put(EEPROM_ENERGY_OFFSET + i*sizeof(int32_t), (int32_t)0);
    }
  }
  byte modbusID = EEPROM.read(EEPROM_ID_OFFSET);
  for(byte i = 0; i < ANALOG_INPUTS_NUM; ++i)
  {
    EEPROM.get(EEPROM_ENERGY_OFFSET + i*sizeof(registers.watt_hour[i]), registers.watt_hour[i]);
    EmonLibCM_setWattHour(i, registers.watt_hour[i]);
  }
/*
 * 
 */
  EmonLibCM_SetADC_VChannel(0, 138.2434);                   // ADC Input channel, voltage calibration - for Ideal UK Adapter = 268.97 
  EmonLibCM_SetADC_IChannel(1, 3.7037, 0.966);              // ADC Input channel, current calibration, phase calibration
  EmonLibCM_SetADC_IChannel(2, 3.7037, 0.966);              //  The current channels will be read in this order

  EmonLibCM_setADC(10, 104);                                // ADC Bits (10 for emonTx & Arduino except Due=12 bits, ADC Duration 104 us for 16 MHz operation)
  EmonLibCM_ADCCal(5.00);                                   // ADC Reference voltage, (3.3 V for emonTx,  5.0 V for Arduino)
  
  EmonLibCM_cycles_per_second(50);                         // mains frequency 50Hz, 60Hz
  EmonLibCM_datalog_period(10);                            // period of readings in seconds - normal value for emoncms.org
  
  EmonLibCM_min_startup_cycles(10);                        // number of cycles to let ADC run before starting first actual measurement

  EmonLibCM_TemperatureEnable(false); 
  ser_println("Init: ", registers.watt_hour[0], ", ", registers.watt_hour[1]);

  EmonLibCM_Init();
 
  Serial.begin(RS485_SERIAL_BAUNDRATE, RS485_SERIAL_CONFIG);

  modbusSlave.setID(modbusID);
  modbusSlave.begin( 0 );

  ser_println("Starting ID:", modbusID, "", "");
}

#ifdef DEBUG_LED_ENABLED
uint32_t lastMillisLed = 0;
#endif
uint32_t lastMillisStore = 0;

void loop()
{
    if (EmonLibCM_Ready())   
    {
      registers.voltage = (uint16_t)((EmonLibCM_getVrms() + 0.5f) * 10.0f);
      registers.frequency = (uint16_t)(EmonLibCM_getLineFrequency() * 10.0f);
      ser_println( "Vrms: ", registers.voltage, " V", "");
        
      for(byte i = 0; i < ANALOG_INPUTS_NUM; ++i)
      {
        registers.current[i] = EmonLibCM_getIrms(i)*1000.0f;
        ser_println(i+1, ". IRms: ", registers.current[i], " A/1000");
  
        registers.power[i] = (uint16_t)(EmonLibCM_getRealPower(i));
        ser_println(i+1, ". P: ", registers.power[i], " W");
        
        registers.watt_hour[i] = EmonLibCM_getWattHour(i);
      }
        
      if( calcTimestampDiff(lastMillisStore, millis()) > 3600)
      {
        lastMillisStore = millis();
        for(byte i = 0; i < ANALOG_INPUTS_NUM; ++i)
        {
          EEPROM.put(EEPROM_ENERGY_OFFSET + i*sizeof(registers.watt_hour[i]), registers.watt_hour[i]);
        }
      }
    }
    int res = modbusSlave.poll( (uint16_t*)&registers, sizeof(Registers)/sizeof(int16_t), registerWasRead );
    if(res != 0)
    {
      ser_println("Response send: ", res, "", "");
    }
}

