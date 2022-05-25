
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

#define SOFT_VER F("1.4.0")

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
  // 0 - No timer, input toggles output
  // >0 - If on will be off after x seconds
  uint32_t      timers[NUM_IOS];
} boardSettings;

static const uint8_t TOPIC_ID_START_INDEX = 6;

static const uint8_t TOPIC_CMD_CHANNEL_INDEX = 23;
char outputCommandTopic[] = { "RELIO/\0\0\0\0\0\0\0\0/cmd/out/+\0" };

static const uint8_t TOPIC_IN_STATE_CHANNEL_INDEX = 24;
char inputStateTopic[]    = { "RELIO/\0\0\0\0\0\0\0\0/state/in/ \0"  };

static const uint8_t TOPIC_OUT_STATE_CHANNEL_INDEX = 25;
char outputStateTopic[]   = { "RELIO/\0\0\0\0\0\0\0\0/state/out/ \0"  };

char clientId[] = { "RELIO_\0\0\0\0\0\0\0\0\0"  };

// Update these with values suitable for your network.
byte mac[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static  uint8_t  inputCounters[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  byte     inputsState = 0;
static  byte     inputsStateToPublish = 0xFF;
static  byte     lastinputsState = 0;
static  byte     outputsState = 0;
static  byte     outputsStateToPublish = 0xFF;
static  uint32_t outputsTimer[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  uint32_t hwResetCount = 0;

void setOutputState(int index, bool state)
{
  byte currentState = outputsState;
  if(state)
  {
    set_bit(outputsState, index);
  }
  else
  {
    clear_bit(outputsState, index);
    // Output is off, reset timer
    outputsTimer[index] = 0;
  }
  if(outputsState != currentState)
  {
    set_bit(outputsStateToPublish, index);
    //Serial.print("O: ");
    //Serial.println(outputsState, BIN);
    MCP27008_write(sWire, MCP27008_ADRESS, outputsState);

    if(get_bit(outputsState, index))
    {
        outputsTimer[index] = boardSettings.timers[index] * 1000; // Convert to miliseconds
    }
  }
}
void toggleOutputState(int index)
{
  setOutputState(index, !get_bit(outputsState, index));
}
void callback(char* topic, byte* payload, unsigned int length)
{
  int8_t relayPin = 0;
  Serial.print(F("Rcv: "));
  Serial.println(topic);
//  Serial.print("[");
//  int i=0;
//  for (i=0;i<length;i++) {
//    Serial.print((char)payload[i]);
//  }
//  Serial.println("]");

  relayPin = topic[TOPIC_CMD_CHANNEL_INDEX] - '0';

  if(relayPin < 1 || relayPin > NUM_IOS)
  {
    Serial.println(F("Wrg pin"));
    return;
  }
  if(length == 2 && (memcmp(payload, "ON", 2) == 0 || memcmp(payload, "on", 2) == 0))
  {
    setOutputState(relayPin-1, true );
  }
  else if(length == 3 && (memcmp(payload, "OFF", 3) == 0 || memcmp(payload, "off", 3) == 0))
  {
    setOutputState(relayPin-1, false );
  }
  else if(length == 6 && (memcmp(payload, "TOGGLE", 6) == 0 || memcmp(payload, "toggle", 6) == 0))
  {
    toggleOutputState(relayPin-1);
  }
  else
  {
    Serial.println(F("Wrg payload"));
  }
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

    // Check if timer elapsed if configured
    if(outputsTimer[i] > 0)
    {
      // Is it time to turn off output?
      if(outputsTimer[i] <= INPUT_CHECK_MS)
      {
        setOutputState(i, false );
      }
      else
      {
        outputsTimer[i] -= INPUT_CHECK_MS;
      }
    }
    
    if(get_bit(inputsState, i) != get_bit(lastinputsState, i))
    {
      if(get_bit(inputsState, i))
      {
        toggleOutputState(i);
      }
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
  byte currentOutputsState = outputsState;
  outputsStateToPublish = 0;
  interrupts();
  
  if(currentOutputsStateToPublish == 0)
    return;
  
  for(byte i = 0; i < NUM_IOS; ++i)
  {
    if(get_bit(currentOutputsStateToPublish, i))
    {
      outputStateTopic[TOPIC_OUT_STATE_CHANNEL_INDEX] = i + '1';
      if(get_bit(currentOutputsState, i))
      {
        publishMsg(client, (const char*)outputStateTopic, "ON");
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
  Serial.print(F("RELIO ver: "));
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
    byteToHexStr(mac[i+2], outputCommandTopic + (TOPIC_ID_START_INDEX + i*2));
    byteToHexStr(mac[i+2], inputStateTopic + (TOPIC_ID_START_INDEX + i*2));
    byteToHexStr(mac[i+2], outputStateTopic + (TOPIC_ID_START_INDEX + i*2));
    byteToHexStr(mac[i+2], clientId + (TOPIC_ID_START_INDEX + i*2));
  }
  Serial.println(outputCommandTopic);
  Serial.println(inputStateTopic);
  Serial.println(outputStateTopic);
  Serial.println(clientId);
  
  Serial.print(F("Timer: "));
  for(byte i = 0; i < NUM_IOS; ++i)
  {
    Serial.print(boardSettings.timers[i]);
    if(i+1 < NUM_IOS)Serial.print(F(", "));
    else Serial.println(F(""));
  }
  
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
  
  MCP27008_setup(sWire, MCP27008_ADRESS);
  MCP27008_write(sWire, MCP27008_ADRESS, outputsState);

  mqttClient.setCallback(callback);

  MsTimer2::set(2, readInputsInterruptHandler);
  MsTimer2::start();
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
        Serial.print(F(" sub= "));
        Serial.println(outputCommandTopic);
        mqttClient.subscribe(outputCommandTopic);
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

bool parseTimers(const char* reqStr, uint32_t* timersArray)
{
  int  iArray;
  bool result = false;
  char formName[] = { "_t =\0" };
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
  if(parseTimers(reqStr, boardSettings.timers))
  {
    Serial.print(F("Received: timers\t:"));
    for(byte i = 0; i < NUM_IOS; ++i)
    {
      Serial.print(boardSettings.timers[i]);
      if(i<7)Serial.print(F(","));
    }
    Serial.println(F(""));
    result = true;
  }
  return result;
}
void addCustomForms(EthernetClient& client)
{
  client.println(F("\n\tTimeout in seconds:"));
  for(byte i = 0; i < NUM_IOS; ++i)
  {
    client.print(F("\t  Channel "));client.print(i+1);
    client.print(F(" : "));
    if(get_bit(outputsState, i))
      client.print(F("ON"));
    else
      client.print(F("OFF"));
    client.print(F("\t<input type=\"text\" name=\"_t"));client.print(i+1); client.print(F("\"  maxlength=3 size=3 value=\"")); 
    client.print(boardSettings.timers[i]);
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
  client.print(F("\tVersion:\t\t"));
  client.println(SOFT_VER);
  client.print(F("\tEthernet resets:\t"));
  client.println(hwResetCount);
}

static uint32_t maintainLastMillis = 0, modCheckLastMillis = 0;
static byte maintainRes, connectionFlag = 0;
static bool ethernedConfigured = false;
void loop()
{
  if(!ethernedConfigured)
  {
    // Perform HW reset
    digitalWrite(ETH_SHIELD_RESET_PIN, LOW);
    delay(10);
    digitalWrite(ETH_SHIELD_RESET_PIN, HIGH);
    
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

    ethernedConfigured = true;
  }
  if(calcTimestampDiff(modCheckLastMillis, millis()) >= 1000)
  {
    modCheckLastMillis = millis();
 
    if(!(uint32_t)Ethernet.localIP())
    {
      Serial.println("Ethernet reset detected");
      ethernedConfigured = false;
      hwResetCount++;
      return;
    }
  }
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

