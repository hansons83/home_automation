
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
  uint32_t     open_time[4];  // open time in miliseconds
  uint32_t     close_time[4]; // close time in miliseconds
  uint32_t     tilt_time[4];  // tilt time in miliseconds
} boardSettings;

static const uint8_t TOPIC_ID_START_INDEX = 8;

static const uint8_t TOPIC_POS_CMD_CHANNEL_INDEX = 35;
char posCommandTopic[]   = { "COVERIO/\0\0\0\0\0\0\0\0/command/position/+\0" };

static const uint8_t TOPIC_TILT_CMD_CHANNEL_INDEX = 30;
char tiltCommandTopic[]  = { "COVERIO/\0\0\0\0\0\0\0\0/command/tilt/+\0" };

static const uint8_t TOPIC_POS_STATE_CHANNEL_INDEX = 32;
char posStateTopic[]     = { "COVERIO/\0\0\0\0\0\0\0\0/state/position/ \0" };

static const uint8_t TOPIC_TILT_STATE_CHANNEL_INDEX = 28;
char tiltStateTopic[]    = { "COVERIO/\0\0\0\0\0\0\0\0/state/tilt/ \0"  };

char clientId[]          = { "COVERIO_\0\0\0\0\0\0\0\0\0" };

// Update these with values suitable for your network.
byte mac[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static const uint8_t NUM_IOS = 8;
static const uint8_t INPUT_HIGH_STATE = 0xFF;
static const uint8_t INPUT_LOW_STATE = 0x00;
static const uint8_t INPUT_PINS_START = 2;
static const uint8_t MCP27008_ADRESS = 0x27;
static const uint8_t PCF8591_ADRESS = 0x48;

static  uint8_t  inputCounters[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  byte     inputsState = 0;
static  byte     inputsStateToPublish = 0;
static  uint32_t lastinputsChange[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  byte     lastinputsState = 0;

static const int8_t   MOVING_UP = -1;
static const int8_t   MOVING_DOWN = 1;
static const int8_t   MOVING_STOP = 1;

static const int   MOVE_RANGE = 100;

static  int8_t outputsDir[] = { 0, 0, 0, 0 };
static  int blindsPos[] = { 0, 0, 0, 0 };
static  int blindsTiltPos[] = { 0, 0, 0, 0 };

static  int8_t outputsExpectedPos[] = { 0, 0, 0, 0 };
static  int8_t outputsExpectedTilt[] = { 0, 0, 0, 0 };

static  uint8_t outputsState = 0;
static  byte    outputsStateToPublish = 0;

#define STATE_MACHINE_STEP_TIME 2

enum ChannelState
{
  CHANNEL_ENTER_IDLE,
  CHANNEL_MONITOR_IDLE,
  CHANNEL_ENTER_POS,
  CHANNEL_MONITOR_POS,
  CHANNEL_ENTER_TILT,
  CHANNEL_MONITOR_TILT,
};

enum ChannelDir
{
  ChannelDir_None = 0,
  ChannelDir_Up = 1,
  ChannelDir_Down = 2
};

const char* stateNames[] = 
{
  "ENTER_IDLE",
  "MONITOR_IDLE",
  "ENTER_POS",
  "MONITOR_POS",
  "ENTER_TILT",
  "MONITOR_TILT",
};

struct BlindState;
typedef void (*blindStateHandler)(uint8_t chInd, BlindState& state);
struct BlindState
{
  bool   stopRequested;
  
  int8_t currentTilt;
  int8_t currentPos;

  int8_t expectedPos;
  int8_t expectedTilt;
  
  int32_t tilt_time;
  int32_t move_time;

  uint32_t          stateEnterMs;
  ChannelState      currentState;
  
} blindStateArray[4];

void setChannelMoveDir(uint8_t chInd, ChannelDir dir)
{
  byte expectedtState = outputsState;

  expectedtState = (expectedtState & ~(0x03 << chInd)) | (dir << chInd);

  Serial.print("O: ");
  Serial.println(expectedtState, BIN);
  if(outputsState != expectedtState)
  {
    outputsState = expectedtState;
    //set_bit(outputsStateToPublish, index);
    MCP27008_write(sWire, MCP27008_ADRESS, outputsState);
  }
}

//void setOutputState(int index, bool state);

bool upButtonPressed(uint8_t chInd)
{
    // Check if button was presed
  if(get_bit(inputsState, chInd*2) != get_bit(lastinputsState, chInd*2))
  {
    if(get_bit(inputsState, chInd*2))
    {
       return true;
    }
  }
  return false;
}
bool downButtonPressed(uint8_t chInd)
{
    // Check if button was presed
  if(get_bit(inputsState, chInd*2+1) != get_bit(lastinputsState, chInd*2+1))
  {
    if(get_bit(inputsState, chInd*2+1))
    {
       return true;
    }
  }
  return false;
}
bool upButtonReleased(uint8_t chInd)
{
    // Check if any button was presed
  if(get_bit(inputsState, chInd*2) != get_bit(lastinputsState, chInd*2))
  {
    if(!get_bit(inputsState, chInd*2))
    {
       return true;
    }
  }
  return false;
}
bool downButtonReleased(uint8_t chInd)
{
    // Check if any button was presed
  if(get_bit(inputsState, chInd*2+1) != get_bit(lastinputsState, chInd*2+1))
  {
    if(!get_bit(inputsState, chInd*2+1))
    {
       return true;
    }
  }
  return false;
}

#define enterState(chInd, expectedState) \
{ \
    if(blindStateArray[chInd].currentState != expectedState) \
    { \
      { \
        Serial.print(F("Blind: ")); \
        Serial.print(chInd); \
        Serial.print(F(", new state: ")); \
        Serial.println(stateNames[expectedState]); \
      } \
      blindStateArray[chInd].stateEnterMs = millis(); \
      blindStateArray[chInd].currentState = expectedState; \
    } \
} while(0)

int32_t calcMoveTime()
{
  
}
void sendPos(uint8_t chInd)
{
//  publishMsg(, posStateTopic, );
}
void sendTilt(uint8_t chInd)
{
//  publishMsg(, tiltStateTopic, );
}

// Nacisniecie gora < 3 sec - Jesli tilt != 0 ustaw expected tilt na current tilt - 25, rozpocznij tilt.
// Nacisniecie góra > 3 sec - Ustaw expected pos na 0, rozpocznij podnoszenie
// Puszczenie góra - Jeśli w monitorowanie tilt zatrzymaj.
// Komenda OPEN - ustaw expected pos i tilt na 0 rozpocznij tilt

// Nacisniecie gora - Jesli tilt != 100 ustaw expected tilt na current tilt + 25, rozpocznij tilt.
// Nacisniecie dol - Jeśli tilt jest inny niż 100, ustaw expected pos i tilt na 100 rozpocznij tilt. Jeśli stan != idle zatrzymaj.
// Puszczenie dol - Jeśli w monitorowanie tilt zatrzymaj.
// Komenda CLOSE - ustaw expected pos i tilt na 100 rozpocznij tilt
/**
 * Wejście do stany bezczynności
 */
void idleEnter(uint8_t chInd, uint8_t elapsedMs)
{
  BlindState& state = blindStateArray[chInd];
  
  Serial.print(F("Blind: ")); Serial.print(chInd);
  Serial.print(F(", Pos: ")); Serial.print(state.currentPos);
  Serial.print(F(", Tilt: ")); Serial.println(state.currentTilt);

  sendPos(chInd);
  sendTilt(chInd);
  setChannelMoveDir(chInd, ChannelDir_None);
  state.stopRequested = false;
  enterState(chInd, CHANNEL_MONITOR_IDLE);
}
/*
 * Monitorowanie stanu bezczynności
 */
void idleMonitor(uint8_t chInd, uint8_t elapsedMs)
{
  BlindState& state = blindStateArray[chInd];
  // Do nothing if we are not at least 1000ms in idle.
  if(calcTimestampDiff(state.stateEnterMs, millis()) < 1000)
  {
    return;
  }
  if(upButtonPressed(chInd))
  {
    Serial.print("Blind up: "); Serial.println(chInd);
    state.expectedPos = 0;
    enterState(chInd, CHANNEL_ENTER_POS); 
  }
  else if(downButtonPressed(chInd))
  {
    Serial.print("Blind down: "); Serial.println(chInd);
    state.expectedPos = 100;
    enterState(chInd, CHANNEL_ENTER_POS); 
  }
}
/*
 * Wejscie w stan zmiany pozycji
 */
void posEnter(uint8_t chInd, uint8_t elapsedMs)
{
  BlindState& state = blindStateArray[chInd];
  
  state.move_time =  ((int32_t)boardSettings.open_time[chInd] * 
    abs(state.currentPos - state.expectedPos)) / (int32_t)100;
    
  if(state.expectedPos < state.currentPos)
  {
    if(state.currentTilt != 0)
    {
      state.expectedTilt = 0;
      enterState(chInd, CHANNEL_ENTER_TILT);
      return;
    }
    setChannelMoveDir(chInd, ChannelDir_Up);
  }
  else if(state.expectedPos > state.currentPos)
  {
    if(state.currentTilt != 100)
    {
      state.expectedTilt = 100;
      enterState(chInd, CHANNEL_ENTER_TILT);
      return;
    }
    setChannelMoveDir(chInd, ChannelDir_Down);
  }
  else
  {
    enterState(chInd, CHANNEL_ENTER_IDLE);
    return;
  }
  Serial.print(F("Move time: ")); Serial.println(state.move_time);
  enterState(chInd, CHANNEL_MONITOR_POS);
}
/*
 * Monitorowanie stanu zmiany pozycji
 */
void posMonitor(uint8_t chInd, uint8_t elapsedMs)
{
  BlindState& state = blindStateArray[chInd];
  /*if(upButtonPressed(chInd) || downButtonPressed(chInd) || state.stopRequested)
  {
    enterState(chInd, CHANNEL_ENTER_IDLE);
    return;
  }*/
  
  state.move_time -= STATE_MACHINE_STEP_TIME;
  if(state.move_time <= 0 )
  {
    state.currentPos = state.expectedPos;
    state.move_time = 0;
    
    if(state.currentTilt != state.expectedTilt)
    {
      enterState(chInd, CHANNEL_ENTER_TILT);
    }
    else
    {
      enterState(chInd, CHANNEL_ENTER_IDLE);
    }
  }
}
/*
 * Wejscie do stanu zmiany kąta lameli
 */
void tiltEnter(uint8_t chInd, uint8_t elapsedMs)
{
  BlindState& state = blindStateArray[chInd];
  
  state.tilt_time = ((int32_t)boardSettings.tilt_time[chInd] * 
  abs(state.currentTilt - state.expectedTilt)) / (int32_t)100;
    
  if(state.expectedTilt < state.currentTilt)
  {
    setChannelMoveDir(chInd, ChannelDir_Up);
  }
  else if(state.expectedTilt > state.currentTilt)
  {
    setChannelMoveDir(chInd, ChannelDir_Down);
  }
  else
  {
    enterState(chInd, CHANNEL_ENTER_IDLE);
    return;
  }
  Serial.print(F("Tilt time: ")); Serial.println(state.tilt_time);
  enterState(chInd, CHANNEL_MONITOR_TILT);
}
/*
 * Monitorowanie stanu zmiany kąta lameli
 */
void tiltMonitor(uint8_t chInd, uint8_t elapsedMs)
{
  BlindState& state = blindStateArray[chInd];
  /*if(upButtonPressed(chInd) || downButtonPressed(chInd) || 
      upButtonReleased(chInd) || downButtonReleased(chInd) || state.stopRequested)
  {
    enterState(chInd, CHANNEL_ENTER_IDLE);
    return;
  }*/
  
  state.tilt_time -= STATE_MACHINE_STEP_TIME;
  if(state.tilt_time <= 0 )
  {
    state.currentTilt = state.expectedTilt;
    state.tilt_time = 0;
    
    if(state.currentPos != state.expectedPos)
    {
      enterState(chInd, CHANNEL_ENTER_POS);
    }
    else
    {
      enterState(chInd, CHANNEL_ENTER_IDLE);
    }
  }
}
/*
 * 
 */
void executeStateMachine(uint8_t elapsedMs)
{
  for(int i = 0; i < 4; ++i)
  {
    switch(blindStateArray[i].currentState)
    {
      case CHANNEL_ENTER_IDLE:
        idleEnter(i, elapsedMs);
      break;
      case CHANNEL_MONITOR_IDLE:
        idleMonitor(i, elapsedMs);
      break;
      case CHANNEL_ENTER_POS:
        posEnter(i, elapsedMs);
      break;
      case CHANNEL_MONITOR_POS:
        posMonitor(i, elapsedMs);
      break;
      case CHANNEL_ENTER_TILT:
        tiltEnter(i, elapsedMs);
      break;
      case CHANNEL_MONITOR_TILT:
        tiltMonitor(i, elapsedMs);
      break;
      default:
        Serial.print(F("Nieobsługiwany stan: "));
      break;
    }
  }
}
/*
//Executed when waiting for input
void blindIdle(uint8_t chInd, BlindState& state)
{
  // Do nothing if we are not at least 500ms in idle.
  if(calcTimestampDiff(state.stateEnterMs, millis()) < 500)
  {
    return;
  }
  if(state.tilt_time)
  {
    enterState(state, blindEnterMove, "blindEnterMove");
    return;
  }
  if(state.tilt_time)
  {
    enterState(state, blindEnterMove, "blindEnterMove");
  }
}
void blindTiltMonitor(uint8_t chInd, BlindState& state, uint8_t elapsedMs)
{
  state.tilt_time -= STATE_MACHINE_STEP_TIME;
  if(state.tilt_time <= 0 )
  {
    state.tilt_time = 0;
    
    if(state.move_time)
    {
      enterState(state, blindEnterTilt, "blindEnterTilt");
    }
    else
    {
      enterState(state, blindStop, "blindStop");
    }
  }
}
void blindMoveMonitor(uint8_t chInd, BlindState& state, uint8_t elapsedMs)
{
  state.move_time -= STATE_MACHINE_STEP_TIME;
  if(state.move_time <= 0 )
  {
    state.move_time = 0;
    
    if(state.tilt_time)
    {
      enterState(state, blindTiltMonitor, "blindMonitorTilt");
    }
    else
    {
      enterState(state, blindIdle, "blindIdle");
    }
  }
}
*/
/*
 * 
 */
/*byte PCF8591_analogRead(byte ind)
{
  uint8_t error;
  Wire.beginTransmission(PCF8591_ADRESS);
  Wire.write(ind);
  error = Wire.endTransmission();
  if(error != 0)
  {
    Serial.print(F("PCF read er: "));
    Serial.println(error);
    return 0;
  }
  Wire.requestFrom(PCF8591_ADRESS, 2);
  Wire.read();
  return Wire.read();
}*/

void readInputsInterruptHandler()
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
      set_bit(inputsState , i);
    }
    else if(inputCounters[i] == INPUT_LOW_STATE)
    {
      clear_bit(inputsState, i);
    }
    if(get_bit(inputsState, i) != get_bit(lastinputsState, i))
    {
      lastinputsChange[i] = millis();
      /*if(!get_bit(boardSettings.mode, i))
      {
        if(get_bit(inputsState, i))
        {
          toggleOutputState(i);
        }
      }
      else
      {  
        set_bit(inputsStateToPublish, i);
      }*/
    }
  }

  executeStateMachine(STATE_MACHINE_STEP_TIME);
  
  lastinputsState = inputsState;
}
/*
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
  }
  if(outputsState != currentState)
  {
    set_bit(outputsStateToPublish, index);
    //Serial.print("O: ");
    //Serial.println(outputsState, BIN);
    MCP27008_write(sWire, MCP27008_ADRESS, outputsState);
  }
}
void toggleOutputState(int index)
{
  setOutputState(index, !get_bit(outputsState, index));
}
*/
void callback(char* topic, byte* payload, unsigned int length)
{
  int8_t channelInd = 0;
  int8_t value;
  Serial.print(F("Rcv: "));
  Serial.println(topic);
  Serial.print("[");
  int i=0;
  for (i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println("]");

  /*relayPin = topic[TOPIC_CMD_CHANNEL_INDEX] - '0';

  if(relayPin < 1 || relayPin > NUM_IOS)
  {
    Serial.println(F("Wrg channel"));
    return;
  }
  if(length == 4 && memcmp(payload, "OPEN", 4) == 0)
  {
  }
  else if(length == 5 && memcmp(payload, "CLOSE", 5) == 0)
  {
  }
  else if(length == 4 && memcmp(payload, "STOP", 4) == 0)
  {
    blindStateArray[channelInd].stopRequested = true;
  }
  else
  {
    value = atoi((const char*)payload);
  }
  if(bright < 0 || bright > 100)
  {
    Serial.println(F("Wrong VAL"));
    return;
  }*/
}

void publishMsg(PubSubClient& client, const char* topic, const char* payload)
{
  Serial.print(F("Pub: "));
  Serial.print(topic);
  Serial.print(F(": "));
  Serial.println(payload);

  client.publish((const char*)topic, payload, true);
}
/*void checkOutputsAndPublish(PubSubClient& client)
{
  noInterrupts();
  byte currentOutputsStateToPublish = outputsStateToPublish;
  byte currentOutputsState = outputsState;
  outputsStateToPublish = 0;
  interrupts();
  
  if(currentOutputsStateToPublish == 0)
    return;
  
  for(uint8_t i = 0; i < NUM_IOS; ++i)
  {
    if(get_bit(currentOutputsStateToPublish, i))
    {
      //outputStateTopic[TOPIC_OUT_STATE_CHANNEL_INDEX] = i + '1';
      //Serial.print(outputStateTopic);
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
}*/
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
    /*if(get_bit(boardSettings.mode, i) && get_bit(currentInputsStateToPublish, i))
    {
      inputStateTopic[TOPIC_IN_STATE_CHANNEL_INDEX] = i + '1';
      Serial.print(inputStateTopic);
      if(get_bit(currentInputsState, i))
      {
        publishMsg(client, (const char*)inputStateTopic, "ON");
      }
      else
      {
        publishMsg(client, (const char*)inputStateTopic, "OFF");
      }
    }*/
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial.print(F("BLINDSIO ver: "));
  Serial.println(SOFT_VER);
 
  for(uint8_t i = 0; i < NUM_IOS; ++i)
  {
    pinMode(INPUT_PINS_START + i, INPUT_PULLUP);      // sets the switch sensor digital pin as input
  }
  
  pinMode(ETH_SHIELD_RESET_PIN, OUTPUT);

  if(EEPROM.read(EEPROM_VERSION_OFFSET) != EEPROM_VERSION)
  {
    Serial.println(F("Clearing EEPROM!"));
    memset(&boardSettings, 0, sizeof(boardSettings));
  
    for(uint8_t i = 0; i < 4; ++i)
    {
      boardSettings.open_time[i] = 6000;
      boardSettings.close_time[i] = 6000;
      boardSettings.tilt_time[i] = 2000;
    }
    EEPROM.write(EEPROM_VERSION_OFFSET, EEPROM_VERSION);
    EEPROM.put(EEPROM_SETTINGS_OFFSET, boardSettings);
  }
  
  EEPROM.get(EEPROM_SETTINGS_OFFSET, boardSettings);
  
  getMacAddress(ds2401, mac);
    
  for (uint8_t i = 0; i < 4; i++)
  {
    byteToHexStr(mac[i+2], clientId + (TOPIC_ID_START_INDEX + i*2));
  }
  memcpy(posCommandTopic+TOPIC_ID_START_INDEX, clientId+TOPIC_ID_START_INDEX, 8);
  memcpy(tiltCommandTopic+TOPIC_ID_START_INDEX, clientId+TOPIC_ID_START_INDEX, 8);
  memcpy(posStateTopic+TOPIC_ID_START_INDEX, clientId+TOPIC_ID_START_INDEX, 8);
  memcpy(tiltStateTopic+TOPIC_ID_START_INDEX, clientId+TOPIC_ID_START_INDEX, 8);

  Serial.println(posCommandTopic);
  Serial.println(tiltCommandTopic);
  Serial.println(posStateTopic);
  Serial.println(tiltStateTopic);
  Serial.println(clientId);
  
  for(uint8_t i = 0; i < 4; ++i)
  {
    Serial.print(boardSettings.mqtt.mqtt_ip[i]);
    if(i < 3)Serial.print(".");
  }
  Serial.print(F(":"));
  Serial.println(boardSettings.mqtt.mqtt_port);
  
  Serial.print(boardSettings.mqtt.mqtt_username);
  Serial.print(F(":"));
  Serial.println(boardSettings.mqtt.mqtt_password);
  
  sWire.begin();
  
  MCP27008_setup(sWire, MCP27008_ADRESS);
  MCP27008_write(sWire, MCP27008_ADRESS, outputsState);
  
  MsTimer2::set(2, readInputsInterruptHandler);
  MsTimer2::start();
  
  // Enable eth module.
  digitalWrite(A0, HIGH);

  mqttClient.setCallback(callback);
  Serial.print(F("Ethernet: "));
  //Ethernet.begin(mac, IPAddress(192, 168, 1, 6), IPAddress(255, 255, 255, 0), IPAddress(192, 168, 1, 254));
  while(!Ethernet.begin(mac))
  {
    delay(1000);
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
        Serial.print(posCommandTopic);
        Serial.print(F(", "));
        Serial.println(tiltCommandTopic);
        mqttClient.subscribe(posCommandTopic);
        mqttClient.subscribe(tiltCommandTopic);
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
    //checkOutputsAndPublish(mqttClient);
    checkInputsAndPublish(mqttClient);
    mqttClient.loop();
  }
}
bool httpReqHandler(char* data, uint16_t size)
{
  char *pch, *modeStr;
  byte counter;
  bool retVal = false;
  /*modeStr = strstr(data, "mode=");
  if(modeStr != NULL)
  {
    counter = 0;
    modeStr += 5;
    Serial.print(F("mode: "));
    pch = strtok (modeStr, ",.&");
    while (pch != NULL && counter < 8)
    {
      if(!isdigit(pch[0]))
      {
        break;
      }
      Serial.print(pch);
      if(counter < 7) Serial.print(F(","));
      boardSettings.mode[counter] = atoi(pch);
      // go to next token
      pch = strtok (NULL, ",.&");
      ++counter;
    }
    Serial.println(F(""));
    retVal = true;
  }*/
  return retVal;
}
void httpRespBuilder(EthernetClient& client)
{
  /*client.print(F("Outputs mode: \t"));
  for(uint8_t i = 0; i < NUM_IOS; ++i)
  {
    client.print(boardSettings.mode[i]);
    if(i+1 < NUM_IOS)client.print(F(", "));
  }
  client.println(F("<BR>"));
  
  client.print(F("<H1>State:</H1>"));
  client.print(F("Soft version: \t\t"));
  client.println(SOFT_VER);
  client.print(F("Subscription: \t\t"));
  client.println(outputCommandTopic);
  client.print(F("MQTT connection state: \t"));
  client.println(mqttClient.state());
  client.print(F("Uptime: \t\t"));
  client.println(millis());
  */
}
  
static uint32_t maintainLastMillis = 0;
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
