/*
 Basic MQTT example
*/

#include <SPI.h>
//#include <Wire.h>
#include <Ethernet2.h>
#include <PubSubClient.h>
#include <MsTimer2.h>
#include <OneWire.h>
#include <EEPROM.h>

#define SOFT_VER "1.1.0"

#define SDA_PORT PORTC
#define SDA_PIN 4
#define SCL_PORT PORTC
#define SCL_PIN 5

#define I2C_TIMEOUT 100
#define I2C_FASTMODE 1

#include <utils.hpp>
#include <http_server.hpp>

#include <SoftWire.h>

SoftWire       sWire = SoftWire();
EthernetServer ethServer(80);
EthernetClient remoteClient;
PubSubClient   mqttClient(remoteClient);
OneWire        ds2401(A1);

#define EEPROM_VERSION_OFFSET  0
#define EEPROM_SETTINGS_OFFSET 1
#define EEPROM_VERSION         0x55

#define ETH_SHIELD_RESET_PIN   A0

static struct StoredSettings{
  MqttSettings mqtt;
  uint16_t     time;    // fade in/out time from 2 to 10000 ms, 0 to disable fading
  byte         mode[8]; // 0 - independent input and outputs, 1 - 100 input toggles output to given value.
} boardSettings;

static const uint8_t TOPIC_ID_START_INDEX = 6;

static const uint8_t TOPIC_CMD_CHANNEL_INDEX = 27;
char outputCommandTopic[]     = { "LEDIO/\0\0\0\0\0\0\0\0/command/out/+\0" };

static const uint8_t TOPIC_IN_STATE_CHANNEL_INDEX = 24;
char inputStateTopic[]  = { "LEDIO/\0\0\0\0\0\0\0\0/state/in/ \0"  };

static const uint8_t TOPIC_OUT_STATE_CHANNEL_INDEX = 25;
char outputStateTopic[] = { "LEDIO/\0\0\0\0\0\0\0\0/state/out/ \0"  };

char clientId[] = { "LEDIO_\0\0\0\0\0\0\0\0\0"  };

// Update these with values suitable for your network.
byte mac[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static const uint8_t NUM_IOS = 8;
static const uint8_t INPUT_HIGH_STATE = 0xFF;
static const uint8_t INPUT_LOW_STATE = 0x00;
static const uint8_t INPUT_PINS_START = 2;
static const uint8_t PCA9634_ADDRESS = 0x47;

static  uint8_t inputCounters[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  byte    inputsState = 0;
static  byte    inputsStateToPublish = 0;
static  byte    lastinputsState = 0;


static  float   outputCurrentVal[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  uint8_t outputExpectedVal[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  int8_t  outputStepSign[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  uint8_t outputsStateToPublish = 0;
static  uint8_t LEDOUT[2] = {0, 0};

const unsigned char PROGMEM CIEL8[50] = {
 0, 1, 1, 2, 2, 3, 4, 5, 6, 7, 
  8, 9, 11, 13, 14, 17, 19, 21, 24, 27, 
  30, 33, 37, 41, 45, 49, 54, 59, 64, 69, 
  75, 81, 88, 95, 102, 109, 117, 125, 134, 143, 
  152, 162, 172, 182, 193, 205, 217, 229, 242, 255, 
};
#define PWM_STEP_VALS sizeof(CIEL8)

#define PCA9634_MODE2_NMOS_INV_VAL (0<<4)
#define PCA9634_MODE2_NMOS_OUTDRV_VAL (1<<2)

#define PCA9634_LEDOUT_OFF_VAL (0)
#define PCA9634_LEDOUT_ON_VAL  (1)
#define PCA9634_LEDOUT_PWM_VAL (2)

#define PCA9634_LED0_STATE_SHIFT 0
#define PCA9634_LED1_STATE_SHIFT 2
#define PCA9634_LED2_STATE_SHIFT 4
#define PCA9634_LED3_STATE_SHIFT 6

#define PCA9634_LED4_STATE_SHIFT 0
#define PCA9634_LED5_STATE_SHIFT 2
#define PCA9634_LED6_STATE_SHIFT 4
#define PCA9634_LED7_STATE_SHIFT 6

uint8_t PCA9634_setup(SoftWire& wire)
{
  uint8_t error = 0;
  // mode 1
  wire.beginTransmission( PCA9634_ADDRESS );
  wire.write( 0x00 );
  wire.write( 0x00 );
  error |= wire.endTransmission( );
  // mode 2
  wire.beginTransmission( PCA9634_ADDRESS );
  wire.write( 0x01 );
  wire.write( PCA9634_MODE2_NMOS_INV_VAL | PCA9634_MODE2_NMOS_OUTDRV_VAL );
  error |= wire.endTransmission( );

  // LEDOUT0
  wire.beginTransmission( PCA9634_ADDRESS );
  wire.write( 0x0C );
  wire.write( LEDOUT[0] );
  error |= wire.endTransmission( );
  // LEDOUT1
  wire.beginTransmission( PCA9634_ADDRESS );
  wire.write( 0x0D );
  wire.write( LEDOUT[1] );
  error |= wire.endTransmission( );

  if(error != 0)
  {
    Serial.print(F("PCA error: "));
    Serial.println(error);
  }
  
  return error;
}
uint8_t PCA9634_write_pwm(SoftWire& wire, uint8_t ledInd, uint8_t val)
{
  uint8_t error = 0;
  uint8_t regOffset = (ledInd / 4);
  uint8_t ledOffset = (ledInd % 4)*2;
  uint8_t ledOutVal = LEDOUT[regOffset];
  
  /*Serial.print("Reg offset: ");
  Serial.print(regOffset);
  Serial.print(" Led offset: ");
  Serial.println(ledOffset);
  */
  if(val >= 100) // Write LEDOUT register to turn on led
  {
    ledOutVal = ledOutVal & ~(0x03 << ledOffset) | (0x01 << ledOffset);
  }
  else if(val < 1)
  {
    ledOutVal = ledOutVal & ~(0x03 << ledOffset);
  }
  else
  {
    ledOutVal = (ledOutVal & ~(0x03 << ledOffset)) | (0x02 << ledOffset);
  }
  if(ledOutVal != LEDOUT[regOffset])
  {
    LEDOUT[regOffset] = ledOutVal;
    wire.beginTransmission( PCA9634_ADDRESS );
    wire.write( 0x0C + regOffset);
    wire.write( LEDOUT[regOffset]);
    error |= wire.endTransmission( );
  }
  
  val = pgm_read_byte_near(CIEL8+(val/2));
  wire.beginTransmission( PCA9634_ADDRESS );
  wire.write( 0x02 + ledInd );
  wire.write( val );
  error |= wire.endTransmission( );
    
  if(error != 0)
  {
    Serial.print(F("PCA error: "));
    Serial.println(error);
  }
  return error;
}
/*
uint8_t PCA9634_read(uint8_t led)
{
  uint8_t retVal;
  uint8_t error = 0;
    // mode 0
  Wire.beginTransmission( PCA9634_ADDRESS );
  Wire.write( 0x02 + led );
  error |= Wire.endTransmission( );
  Wire.requestFrom( PCA9634_ADDRESS, 1, 1);
  while(!Wire.available());
  retVal = Wire.read( );

  return retVal;
}
*/
void setOutputState(int ledIndex, uint8_t bright)
{
  noInterrupts();
  if(outputCurrentVal[ledIndex] > bright)
  {
    outputStepSign[ledIndex] = -1;
  }
  else
  { 
    outputStepSign[ledIndex] = 1;
  }
  outputExpectedVal[ledIndex] = bright;
  interrupts();
}
void toggleOutputState(int ledIndex, uint8_t bright)
{
  if(outputCurrentVal[ledIndex] > 0)
  {
    setOutputState(ledIndex, 0);
  }
  else
  {
    setOutputState(ledIndex, bright);
  }
}
void callback(char* topic, byte* payload, unsigned int length)
{
  int8_t ledNr = 0;
  int bright = -1;
  Serial.print(F("Rcv: "));
  Serial.println(topic);
//  Serial.print("[");
//  int i=0;
//  for (i=0;i<length;i++) {
//    Serial.print((char)payload[i]);
//  }
//  Serial.println("]");
  payload[length] = 0;

  ledNr = topic[TOPIC_CMD_CHANNEL_INDEX] - '0';
  
  if(ledNr < 1 || ledNr > NUM_IOS)
  {
    Serial.println(F("Wrong LED"));
    return;
  }
  if(length == 2 && memcmp(payload, "ON", 2) == 0)
  {
    bright = 100;
  }
  else if(length == 3 && memcmp(payload, "OFF", 3) == 0)
  {
    bright = 0;
  }
  else
  {
    bright = atoi((const char*)payload);
  }
  if(bright < 0 || bright > 100)
  {
    Serial.println(F("Wrong VAL"));
    return;
  }
  Serial.print(F("Ch: "));
  Serial.print(ledNr);
  Serial.print(F(", Val: "));
  Serial.println(bright);
  //outputCurrentVal[ledNr-1] = PCA9634_read(ledNr-1);
  //Serial.println(outputCurrentVal[ledNr-1]);
  setOutputState(ledNr-1, bright);
}

void readInputsUpdateOutputsHandler()
{
  for(uint8_t i = 0; i < NUM_IOS; ++i)
  {
    inputCounters[i] <<= 1;
    if(digitalRead(INPUT_PINS_START + i) == LOW)
    {
      inputCounters[i] += 1;
    }
    else
    {
      inputCounters[i] += 0;
    }
    if(inputCounters[i] == INPUT_HIGH_STATE)
    {
      set_bit(inputsState, i);
    }
    else if(inputCounters[i] == INPUT_LOW_STATE)
    {
      clear_bit(inputsState, i);
    }
    if(get_bit(inputsState, i) != get_bit(lastinputsState, i))
    {
      if(boardSettings.mode[i] > 0)
      {
        if(get_bit(inputsState, i))
        {
          toggleOutputState(i, boardSettings.mode[i]);
        }
      }
      else
      { 
        set_bit(inputsStateToPublish, i);
      }
    }

    if(outputCurrentVal[i] != outputExpectedVal[i])
    {
      outputCurrentVal[i] += (200.0f / (float)boardSettings.time) * outputStepSign[i];
      if(outputStepSign[i] > 0)
      {
        if(outputCurrentVal[i] >= outputExpectedVal[i])
        {
          outputCurrentVal[i] = outputExpectedVal[i];
          set_bit(outputsStateToPublish, i);
        }
      }
      else
      {
        if(outputCurrentVal[i] <= outputExpectedVal[i])
        {
          outputCurrentVal[i] = outputExpectedVal[i];
          set_bit(outputsStateToPublish, i);
        }
      }
      PCA9634_write_pwm(sWire, i, outputCurrentVal[i]);
    }

  }
  lastinputsState = inputsState;
}

void publishMsg(PubSubClient& client, const char* topic, const char* payload)
{
  Serial.print(F("Pub: "));
  Serial.print(topic);
  Serial.print(F(": "));
  Serial.println(payload);

  client.publish((const char*)topic, payload, true);
}

void checkOutputsAndPublish(PubSubClient& client)
{
  noInterrupts();
  byte currentOutputsStateToPublish = outputsStateToPublish;  
  outputsStateToPublish = 0;
  interrupts();
  
  if(currentOutputsStateToPublish == 0)
    return;
  
  for(uint8_t i = 0; i < NUM_IOS; ++i)
  {
    if(get_bit(currentOutputsStateToPublish, i))
    {
      outputStateTopic[TOPIC_OUT_STATE_CHANNEL_INDEX] = i + '1';
      if(outputCurrentVal[i] != 0)
      {
        char buffer[7]; 
        itoa((int)outputCurrentVal[i], buffer, 10);
        
        publishMsg(client, (const char*)outputStateTopic, buffer);
      }
      else
      {
        publishMsg(client, (const char*)outputStateTopic, "OFF");
      }
    }
  }
}

void checkInputsAndPublish(PubSubClient& client)
{
  noInterrupts();
  byte currentInputsStateToPublish = inputsStateToPublish;
  byte currentInputsState = inputsState;
  inputsStateToPublish = 0;
  interrupts();
  
  if(currentInputsStateToPublish == 0)
    return;
  
  for(uint8_t i = 0; i < NUM_IOS; ++i)
  {
    if(get_bit(currentInputsStateToPublish, i))
    {
      inputStateTopic[TOPIC_IN_STATE_CHANNEL_INDEX] = i + '1';
      if(get_bit(currentInputsState, i))
      {
        publishMsg(client, (const char*)inputStateTopic, "ON");
      }
      else
      {
        publishMsg(client, (const char*)inputStateTopic, "OFF");
      }
    }
  }
}
/*
void PrintTwoDigitHex (byte b, boolean newline)
{
  Serial.print(b/16, HEX);
  Serial.print(b%16, HEX);
  if (newline) Serial.println();
}
*/
void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial.print(F("LEDIO ver: "));
  Serial.println(SOFT_VER);
 
  for(uint8_t i = 0; i < NUM_IOS; ++i)
  {
    pinMode(INPUT_PINS_START + i, INPUT);      // sets the switch sensor digital pin as input
  }
  
  pinMode(ETH_SHIELD_RESET_PIN, OUTPUT);

  if(EEPROM.read(EEPROM_VERSION_OFFSET) != EEPROM_VERSION)
  {
    Serial.println(F("Clearing EEPROM!"));
    memset(&boardSettings, 0, sizeof(boardSettings));
    boardSettings.time = 200;
    memset(&boardSettings.mode, 100, sizeof(boardSettings.mode));
    
    EEPROM.write(EEPROM_VERSION_OFFSET, EEPROM_VERSION);
    EEPROM.put(EEPROM_SETTINGS_OFFSET, boardSettings);
  }
  
  EEPROM.get(EEPROM_SETTINGS_OFFSET, boardSettings);

  getMacAddress(ds2401, mac);

  for (uint8_t i = 0; i < 4; i++)
  {
    byteToHexStr(mac[i+2], outputCommandTopic + (TOPIC_ID_START_INDEX + i*2));
    byteToHexStr(mac[i+2], inputStateTopic + (TOPIC_ID_START_INDEX + i*2));
    byteToHexStr(mac[i+2], outputStateTopic + (TOPIC_ID_START_INDEX + i*2));
    byteToHexStr(mac[i+2], clientId + (TOPIC_ID_START_INDEX + i*2));
  }
  Serial.println(outputCommandTopic);
  Serial.println(inputStateTopic);
  Serial.println(outputStateTopic);

  Serial.print(F("Mode: "));
  for(uint8_t i = 0; i < NUM_IOS; ++i)
  {
    Serial.print(boardSettings.mode[i]);
    if(i+1 < NUM_IOS)Serial.print(F(", "));
    else Serial.println(F(""));
  }
  
  Serial.print(F("Time: "));
  Serial.println(boardSettings.time);
  
  for(uint8_t i = 0; i < 4; ++i)
  {
    Serial.print(boardSettings.mqtt.mqtt_ip[i]);
    if(i < 3)Serial.print(F("."));
  }
  Serial.print(F(":"));
  Serial.println(boardSettings.mqtt.mqtt_port);
  
  Serial.print(boardSettings.mqtt.mqtt_username);
  Serial.print(F(":"));
  Serial.println(boardSettings.mqtt.mqtt_password);
  
  sWire.begin();

  PCA9634_setup(sWire);
  for(uint8_t i = 0; i < NUM_IOS; ++i)
  {
    PCA9634_write_pwm(sWire, i, outputCurrentVal[i]);
  }

  MsTimer2::set(2, readInputsUpdateOutputsHandler);
  MsTimer2::start();

  // Enable eth module.
  digitalWrite(A0, HIGH);

  mqttClient.setCallback(callback);
  Serial.print(F("Ethernet: "));
  //Ethernet.begin(mac, IPAddress(192, 168, 1, 6), IPAddress(255, 255, 255, 0), IPAddress(192, 168, 1, 254));
  while(!Ethernet.begin(mac))
  {
    delay(5000);
  }
  ethServer.begin();

  Serial.println(Ethernet.localIP()); 
  //Serial.println(Ethernet.subnetMask());
  //Serial.println(Ethernet.gatewayIP());
  //Serial.println(Ethernet.dnsServerIP());
}

static uint32_t lastConnectMillis = 0;
static byte mqttConnectionFlag;
void handleMqttClient()
{
  if (!mqttClient.connected())
  {
    if(mqttConnectionFlag == 0 || calcTimestampDiff(lastConnectMillis, millis()) >= 20000)
    {
      mqttConnectionFlag = 1;
      lastConnectMillis = millis();
      
      mqttClient.setServer(boardSettings.mqtt.mqtt_ip, boardSettings.mqtt.mqtt_port);
      Serial.print(F("MQTT connecting..."));
      if (mqttClient.connect(clientId, boardSettings.mqtt.mqtt_username, boardSettings.mqtt.mqtt_password))
      {
        Serial.print(F(" Connected, sub= "));
        Serial.println(outputCommandTopic);
        mqttClient.subscribe(outputCommandTopic);
      }
      else
      {
        Serial.print(F(" Disconnected, err="));
        Serial.println(mqttClient.state());
      }
    }
  }
  else
  {
    checkOutputsAndPublish(mqttClient);
    checkInputsAndPublish(mqttClient);
    mqttClient.loop();
  }
}

bool httpReqHandler(char* data, uint16_t size)
{
  char *timeStr, *modeStr, *pch;
  byte counter;
  bool retVal = false;
  timeStr = strstr(data, "time=");
  modeStr = strstr(data, "mode=");
  if(timeStr != NULL)
  {
    timeStr += 5;
    Serial.print(F("time: "));
    pch = strtok (timeStr, "&");
    Serial.print(pch);
    Serial.println(F("ms"));
    boardSettings.time = atoi(pch);
    if(boardSettings.time < 2)
    {
      boardSettings.time = 2;
    }
    if(boardSettings.time > 10000)
    {
      boardSettings.time = 10000;
    }
    retVal = true;
  }
  if(modeStr != NULL)
  {
    counter = 0;
    modeStr += 5;
    Serial.print(F("mode: "));
    pch = strtok (modeStr, ".,&");
    while (pch != NULL && counter < 8)
    {
      if(!isdigit(pch[0]))
      {
        break;
      }
      Serial.print(pch);
      if(counter < 7) Serial.print(F(","));
      else Serial.println("");
      boardSettings.mode[counter] = atoi(pch);
      // go to next token
      pch = strtok (NULL, ".,&");
      ++counter;
    }
    retVal = true;
  }
  return retVal;
}
void httpRespBuilder(EthernetClient& client)
{  
  client.print(F("Fade time: \t"));
  client.print(boardSettings.time);
  client.println(F("ms"));
  client.print(F("Channel mode: \t"));
  for(uint8_t i = 0; i < 8; ++i)
  {
    client.print(boardSettings.mode[i]);
    if(i < 7)client.print(",");
  }
  client.println(F(""));
  client.print(F("<H1>State:</H1>"));
  client.print(F("Soft version: \t\t"));
  client.println(SOFT_VER);
  client.print(F("Subscription: \t\t"));
  client.println(outputCommandTopic);
  client.print(F("MQTT connection state: \t"));
  client.println(mqttClient.state());
  client.print(F("Uptime: \t\t"));
  client.println(millis()); 
}
static unsigned long maintainLastMillis = 0;
static byte maintainRes, connectionFlag;
void loop()
{
  if(calcTimestampDiff(maintainLastMillis, millis()) >= 10000)
  {
    maintainLastMillis = millis();
    
    maintainRes = Ethernet.maintain();
    if(maintainRes == 2 || maintainRes == 4)
    {
      connectionFlag = 1;
      Serial.print(F("Ethernet: "));
      Serial.println(Ethernet.localIP()); 
    }
    else
    {
      if(connectionFlag)
      {
        Serial.println(F("Ethernet: disconnected"));
      }
      connectionFlag = 0;
      return;
    }
  }
  
  handleMqttClient();
  
  HttpResult httpRes = httpHandle(ethServer, boardSettings.mqtt, httpReqHandler, httpRespBuilder);
  if(httpRes != HTTP_NO_ACTION)
  {
    EEPROM.put(EEPROM_SETTINGS_OFFSET, boardSettings);
    Serial.println(F("EEPROM updated")); 
    if(httpRes & HTTP_MQTT_CHANGE)
    {
      mqttConnectionFlag = 0;
      mqttClient.disconnect();
      Serial.println(F("MQTT disconnected"));
    }
  }
}

