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

#define SOFT_VER F("1.2.3")

#define SDA_PORT PORTC
#define SDA_PIN 4
#define SCL_PORT PORTC
#define SCL_PIN 5

#define I2C_TIMEOUT 100
#define I2C_FASTMODE 1

#define HTTP_REQ_BUF_SZ 250

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
#define EEPROM_VERSION         0x54

#define ETH_SHIELD_RESET_PIN   A0

static const uint8_t NUM_IOS = 8;
static const uint8_t INPUT_HIGH_STATE = 0xFF;
static const uint8_t INPUT_LOW_STATE = 0x00;
static const uint8_t INPUT_PINS_START = 2;
static const uint8_t PCA9634_ADDRESS = 0x47;

static struct StoredSettings{
  MqttSettings mqtt;
  uint16_t     fade_time[NUM_IOS];  // fade in/out time from 2 to 10000 ms, 0 to disable fading
  uint8_t      bright[NUM_IOS];    // bright when button pressed
} boardSettings;

static const uint8_t TOPIC_ID_START_INDEX = 6;

static const uint8_t TOPIC_CMD_CHANNEL_INDEX = 23;
char outputCommandTopic[]     = { "LEDIO/\0\0\0\0\0\0\0\0/cmd/out/+\0" };

static const uint8_t TOPIC_IN_STATE_CHANNEL_INDEX = 24;
char inputStateTopic[]  = { "LEDIO/\0\0\0\0\0\0\0\0/state/in/ \0"  };

static const uint8_t TOPIC_OUT_STATE_CHANNEL_INDEX = 25;
char outputStateTopic[] = { "LEDIO/\0\0\0\0\0\0\0\0/state/out/ \0"  };

static const uint8_t TOPIC_OUT_BSTATE_CHANNEL_INDEX = 26;
char outputBStateTopic[] = { "LEDIO/\0\0\0\0\0\0\0\0/bstate/out/ \0"  };

char clientId[] = { "LEDIO_\0\0\0\0\0\0\0\0\0"  };

// Update these with values suitable for your network.
byte mac[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static  uint8_t inputCounters[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  byte    inputsState = 0;
static  byte    inputsStateToPublish = 0;
static  byte    lastinputsState = 0;


static  float   outputCurrentVal[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  byte    outputCurrentBright[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  uint8_t outputExpectedVal[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  int8_t  outputStepSign[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  uint8_t outputsStateToPublish = 0;
static  uint8_t LEDOUT[2] = {0, 0};

#define PWM_LED_BRIGHT_MAX_VAL 255
#define PWM_LED_BRIGHT_MIN_VAL 0
/*
const unsigned char PROGMEM CIEL8[256] = {
  0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
  1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 
  2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 
  3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 
  3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 
  4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 
  6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 
  7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 
  9, 9, 9, 10, 10, 10, 10, 10, 11, 11, 
  11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 
  14, 14, 15, 15, 15, 16, 16, 17, 17, 17, 
  18, 18, 19, 19, 19, 20, 20, 21, 21, 22, 
  22, 23, 23, 24, 24, 25, 26, 26, 27, 27, 
  28, 29, 29, 30, 31, 31, 32, 33, 34, 34, 
  35, 36, 37, 38, 38, 39, 40, 41, 42, 43, 
  44, 45, 46, 47, 48, 49, 50, 51, 53, 54, 
  55, 56, 57, 59, 60, 61, 63, 64, 66, 67, 
  69, 70, 72, 73, 75, 77, 79, 80, 82, 84, 
  86, 88, 90, 92, 94, 96, 98, 100, 103, 105, 
  107, 110, 112, 115, 117, 120, 123, 125, 128, 131, 
  134, 137, 140, 143, 147, 150, 153, 157, 160, 164, 
  167, 171, 175, 179, 183, 187, 191, 196, 200, 205, 
  209, 214, 219, 224, 229, 234, 239, 244, 250, 255, 
  255, 255, 255, 255, 255
};
*/
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
  float calc = val;
  calc = calc * calc * 0.003921568627f;////pgm_read_byte_near(CIEL8 + val);
  if(calc > 255)
    calc = 255;

  val = (int)round(calc);
  //Serial.println(val);
  
  if(outputCurrentBright[ledInd] == calc)
  {
    return 0;
  }
  outputCurrentBright[ledInd] = calc;
  
  //Serial.println(calc);

  //Serial.print("Led: "); Serial.print(ledInd); Serial.print(" Write: "); Serial.println(val);
  
  uint8_t error = 0;
  uint8_t regOffset = (ledInd / 4);
  uint8_t ledOffset = (ledInd % 4)*2;
  uint8_t ledOutVal = LEDOUT[regOffset];
  
  /*Serial.print("Reg offset: ");
  Serial.print(regOffset);
  Serial.print(" Led offset: ");
  Serial.println(ledOffset);
  */
  if(val >= PWM_LED_BRIGHT_MAX_VAL) // Write LEDOUT register to turn on led
  {
    ledOutVal = ledOutVal & ~(0x03 << ledOffset) | (0x01 << ledOffset);
  }
  else if(val == PWM_LED_BRIGHT_MIN_VAL)
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
  if(bright != 0)
  {
    set_bit(outputsStateToPublish, ledIndex);
  }
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
  if(length == 2 && (memcmp(payload, "ON", 2) == 0 || memcmp(payload, "on", 2) == 0))
  {
    bright = boardSettings.bright[ledNr-1];
  }
  else if(length == 3 && (memcmp(payload, "OFF", 3) == 0 || memcmp(payload, "OFF", 3) == 0))
  {
    bright = 0;
  }
  else
  {
    bright = atoi((const char*)payload);
  }
  if(bright < PWM_LED_BRIGHT_MIN_VAL || bright > PWM_LED_BRIGHT_MAX_VAL)
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
  bool publishState = false;
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
      if(get_bit(inputsState, i))
      {
        toggleOutputState(i, boardSettings.bright[i]);
      }
    }

    if(outputCurrentVal[i] != outputExpectedVal[i])
    {
      outputCurrentVal[i] += (500.0f / (float)boardSettings.fade_time[i]) * outputStepSign[i];
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
      PCA9634_write_pwm(sWire, i, (byte)outputCurrentVal[i]);
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
  
  for(byte i = 0; i < NUM_IOS; ++i)
  {
    if(get_bit(currentOutputsStateToPublish, i))
    {
      outputStateTopic[TOPIC_OUT_STATE_CHANNEL_INDEX] = i + '1';
      outputBStateTopic[TOPIC_OUT_BSTATE_CHANNEL_INDEX] = i + '1';
      char buffer[7]; 
      itoa((int)outputCurrentVal[i], buffer, 10);
      publishMsg(client, (const char*)outputBStateTopic, buffer);
      
      if(outputExpectedVal[i] == 0)
      {
        publishMsg(client, (const char*)outputStateTopic, "OFF");
      }
      else
      {
        publishMsg(client, (const char*)outputStateTopic, "ON");
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
  
  for(byte i = 0; i < NUM_IOS; ++i)
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

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial.print(F("LEDIO ver: "));
  Serial.println(SOFT_VER);
 
  for(byte i = 0; i < NUM_IOS; ++i)
  {
    pinMode(INPUT_PINS_START + i, INPUT);      // sets the switch sensor digital pin as input
  }
  
  pinMode(ETH_SHIELD_RESET_PIN, OUTPUT);

  if(EEPROM.read(EEPROM_VERSION_OFFSET) != EEPROM_VERSION)
  {
    Serial.println(F("Clearing EEPROM!"));
    memset(&boardSettings, 0, sizeof(boardSettings));
    for(byte i = 0; i < NUM_IOS; ++i)
    {
      boardSettings.fade_time[i] = 500;
      boardSettings.bright[i] = 255;
    }
    
    EEPROM.write(EEPROM_VERSION_OFFSET, EEPROM_VERSION);
    EEPROM.put(EEPROM_SETTINGS_OFFSET, boardSettings);
  }
  
  EEPROM.get(EEPROM_SETTINGS_OFFSET, boardSettings);

  getMacAddress(ds2401, mac);

  for (byte i = 0; i < 4; i++)
  {
    byteToHexStr(mac[i+2], outputCommandTopic + (TOPIC_ID_START_INDEX + i*2));
    byteToHexStr(mac[i+2], inputStateTopic + (TOPIC_ID_START_INDEX + i*2));
    byteToHexStr(mac[i+2], outputStateTopic + (TOPIC_ID_START_INDEX + i*2));
    byteToHexStr(mac[i+2], outputBStateTopic + (TOPIC_ID_START_INDEX + i*2));
    byteToHexStr(mac[i+2], clientId + (TOPIC_ID_START_INDEX + i*2));
  }
  Serial.println(outputCommandTopic);
  Serial.println(inputStateTopic);
  Serial.println(outputStateTopic);
/*
  Serial.print(F("Bright: "));
  for(uint8_t i = 0; i < NUM_IOS; ++i)
  {
    Serial.print(boardSettings.bright[i]);
    if(i+1 < NUM_IOS)Serial.print(F(", "));
    else Serial.println(F(""));
  }
  
  Serial.print(F("Time: "));
  Serial.println(boardSettings.fade_time);
*/
  for(byte i = 0; i < 4; ++i)
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
  for(byte i = 0; i < NUM_IOS; ++i)
  {
    PCA9634_write_pwm(sWire, i, outputCurrentVal[i]);
  }

  MsTimer2::set(2, readInputsUpdateOutputsHandler);
  MsTimer2::start();

  // Enable eth module.
  digitalWrite(A0, HIGH);

  mqttClient.setCallback(callback);
  //Ethernet.begin(mac, IPAddress(192, 168, 1, 6), IPAddress(255, 255, 255, 0), IPAddress(192, 168, 1, 254));
  while(!Ethernet.begin(mac))
  {
    delay(1000);
  }
  ethServer.begin();

  Serial.print(F("Ethernet: "));
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
        Serial.print(F(" sub= "));
        Serial.println(outputCommandTopic);
        mqttClient.subscribe(outputCommandTopic);
        inputsStateToPublish = outputsStateToPublish = 0xFF;
      }
      else
      {
        Serial.print(F(" err= "));
        Serial.println(mqttClient.state());
      }
    }
  }
  else
  {
    checkInputsAndPublish(mqttClient);
    checkOutputsAndPublish(mqttClient);
    mqttClient.loop();
  }
}

CustomHandlers customHandlers = 
{
  .customProcess = processCustomParams,
  .customForms = addCustomForms,
  .mqtt_ptr = &boardSettings.mqtt
};

bool parseFadeTime(const char* reqStr, uint16_t* timersArray)
{
  int  iArray;
  bool result = false;
  char formName[] = { "_f =\0" };
  for(byte i = 0; i < NUM_IOS; ++i)
  {
    formName[2] =  i + '1';
    char* strPtr = strstr(reqStr, formName);
    if(strPtr)
    {
      strPtr += 3;
      if(sscanf(strPtr, "=%d&", &iArray) == 1)
      {
        timersArray[i] = iArray;
          
        result = true;
      }
    }
  }
  return result;
}
bool parseBright(const char* reqStr, uint8_t* timersArray)
{
  int  iArray;
  bool result = false;
  char formName[] = { "_b =\0" };
  for(byte i = 0; i < NUM_IOS; ++i)
  {
    formName[2] =  i + '1';
    char* strPtr = strstr(reqStr, formName);
    if(strPtr)
    {
      strPtr += 3;
      if(sscanf(strPtr, "=%d&", &iArray) == 1)
      {
        timersArray[i] = iArray;
          
        result = true;
      }
    }
  }
  return result;
}
bool processCustomParams(const char* reqStr)
{
  bool result = false;
  if(parseBright(reqStr, boardSettings.bright))
  {
    Serial.print(F("Received: bright\t:"));
    for(byte i = 0; i < 8; ++i)
    {
      Serial.print(boardSettings.bright[i]);
      if(i<7)Serial.print(F(","));
    }
    Serial.println(F(""));
    result = true;
  }
  if(parseFadeTime(reqStr, boardSettings.fade_time))
  {
    Serial.print(F("Received: fade_time\t:"));
    for(int i = 0; i < 8; ++i)
    {
      Serial.print(boardSettings.fade_time[i]);
      if(i<7)Serial.print(F(","));
    }
    Serial.println(F(""));
    result = true;
  }
  return result;
}
void addCustomForms(EthernetClient& client)
{
  client.println(F("\n\t\tFade time(ms)\t\t On brightness (%)"));
  for(byte i = 0; i < NUM_IOS; ++i)
  {
    client.print(F("  Channel "));client.print(i+1);
    client.print(F("\t<input type=\"text\" name=\"_f"));client.print(i+1); client.print(F("\"  maxlength=5 size=5 value=\"")); 
    client.print(boardSettings.fade_time[i]);
    client.print(F("\">"));
    client.print(F("\t\t<input type=\"text\" name=\"_b"));client.print(i+1); client.print(F("\"  maxlength=3 size=3 value=\"")); 
    client.print(boardSettings.bright[i]);
    client.println(F("\">"));
  }

  client.println(F(""));
  client.print(F("<H1>State:</H1>"));
  client.print(F("\tMQTT client id:\t\t"));
  client.println(clientId);
  client.print(F("\tMQTT subscription:\t"));
  client.println(outputCommandTopic);
  client.print(F("\tMQTT connection state:\t"));
  client.println(mqttClient.state());
  client.print(F("\tUptime:\t\t\t"));
  client.println(millis()); 
  client.print(F("\tSoft version:\t\t"));
  client.println(SOFT_VER);
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
  
  HttpResult httpRes = httpHandle2(ethServer, customHandlers, Ethernet.localIP());
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

