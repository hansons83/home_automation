
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

#define SOFT_VER F("1.1.1")

#define SDA_PORT PORTC
#define SDA_PIN 4
#define SCL_PORT PORTC
#define SCL_PIN 5

#define I2C_TIMEOUT 100
#define I2C_FASTMODE 1

#include <utils.hpp>
#include <http_server.hpp>

SoftWire       sWire = SoftWire();
EthernetServer ethServer(80);
EthernetClient remoteClient;
PubSubClient   mqttClient(remoteClient);
OneWire        ds2401(A1);

static const uint8_t EEPROM_VERSION_OFFSET  = 0;
static const uint8_t EEPROM_SETTINGS_OFFSET = 1;
static const uint8_t EEPROM_VERSION         = 0x55;

static const uint8_t ETH_SHIELD_RESET_PIN   = A0;

static const uint8_t INPUT_CHECK_MS = 2;
static const uint8_t NUM_IOS = 8;
static const uint8_t INPUT_HIGH_STATE = 0xFF;
static const uint8_t INPUT_LOW_STATE = 0x00;
static const uint8_t INPUT_PINS_START = 2;
static const uint8_t MCP27008_ADRESS = 0x27;

static struct StoredSettings{
  MqttSettings mqtt;
} boardSettings;

static const uint8_t TOPIC_ID_START_INDEX = 5;
static const uint8_t TOPIC_IN_STATE_CHANNEL_INDEX = 23;
char inputStateTopic[]    = { "INIO/\0\0\0\0\0\0\0\0/state/in/ \0"  };

char clientId[] = { "INIO_\0\0\0\0\0\0\0\0\0"  };

// Update these with values suitable for your network.
byte mac[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static  uint8_t  inputCounters[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  byte     inputsState = 0;
static  byte     inputsStateToPublish = 0xFF;
static  byte     lastinputsState = 0;


void callback(char* topic, byte* payload, unsigned int length)
{
}

void readInputsInterruptHandler()
{
  for(byte i = 0; i < NUM_IOS; ++i)
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
    if(get_bit(lastinputsState, i) != get_bit(inputsState, i))
      set_bit(inputsStateToPublish, i);
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
  Serial.print(F("INIO ver: "));
  Serial.println(SOFT_VER);
 
  for(byte i = 0; i < NUM_IOS; ++i)
  {
    pinMode(INPUT_PINS_START + i, INPUT_PULLUP);      // sets the switch sensor digital pin as input
  }
  
  pinMode(ETH_SHIELD_RESET_PIN, OUTPUT);

  if(EEPROM.read(EEPROM_VERSION_OFFSET) != EEPROM_VERSION)
  {
    Serial.println(F("Clearing EEPROM!"));
    memset(&boardSettings, 0, sizeof(boardSettings));
    EEPROM.write(EEPROM_VERSION_OFFSET, EEPROM_VERSION);
    EEPROM.put(EEPROM_SETTINGS_OFFSET, boardSettings);
  }
  
  EEPROM.get(EEPROM_SETTINGS_OFFSET, boardSettings);

  getMacAddress(ds2401, mac);
  
  for (byte i = 0; i < 4; i++)
  {
    byteToHexStr(mac[i+2], inputStateTopic + (TOPIC_ID_START_INDEX + i*2));
    byteToHexStr(mac[i+2], clientId + (TOPIC_ID_START_INDEX + i*2));
  }
  Serial.println(inputStateTopic);
  Serial.println(clientId);

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
  
  MsTimer2::set(2, readInputsInterruptHandler);
  MsTimer2::start();

  // Enable eth module.
  digitalWrite(A0, HIGH);

  mqttClient.setCallback(callback);
  //Ethernet.begin(mac, IPAddress(192, 168, 1, 6), IPAddress(255, 255, 255, 0), IPAddress(192, 168, 1, 254));
  while(!Ethernet.begin(mac))
  {
    delay(5000);
  }
  ethServer.begin();

  Serial.print(F("Ethernet: "));
  Serial.println(Ethernet.localIP()); 
  //Serial.println(Ethernet.subnetMask());
  //Serial.println(Ethernet.gatewayIP());
  //Serial.println(Ethernet.dnsServerIP());
}

static uint32_t lastConnectMillis = 0;
static byte mqttConnectionFlag = 0;
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
        Serial.println(F(" connected"));
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
    mqttClient.loop();
  }
}
bool processCustomParams(char* data, uint16_t size)
{
  return false;
}
CustomHandlers customHandlers = 
{
  .customProcess = processCustomParams,
  .customForms = addCustomForms,
  .mqtt_ptr = &boardSettings.mqtt
};

void addCustomForms(EthernetClient& client)
{
  client.println(F(""));
  client.print(F("<H1>State:</H1>"));
  client.print(F("\tMQTT client id:\t\t"));
  client.println(clientId);
  client.print(F("\tMQTT connection state:\t"));
  client.println(mqttClient.state());
  client.print(F("\tUptime:\t\t\t"));
  client.println(millis()); 
  client.print(F("\tVersion:\t\t"));
  client.println(SOFT_VER);
}

static uint32_t maintainLastMillis = 0;
static byte maintainRes, connectionFlag = 0;
void loop()
{
  if(calcTimestampDiff(maintainLastMillis, millis()) >= 5000)
  {
    maintainLastMillis = millis();
    
    maintainRes = Ethernet.maintain();
    Serial.print(F("Ethernet: maintain: ")); Serial.println(maintainRes);
    
    Serial.print(F("MQTT status: "));
    Serial.println(mqttClient.state());
     
    //Serial.print(F("Ethernet: "));
    //Serial.println(Ethernet.localIP()); 
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

