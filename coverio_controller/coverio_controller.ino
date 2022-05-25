  
/*
 Basic MQTT example
*/
//#define COVERIO_DEBUG_MODE

#include <SPI.h>
//#include <Wire.h>
#include <Ethernet2.h>
#include <PubSubClient.h>
#include <MsTimer2.h>
#include <OneWire.h>
#include <EEPROM.h>


#define SOFT_VER F("1.4.0")

#ifdef COVERIO_DEBUG_MODE
#define SerialPrint(x) Serial.print(x)
#define SerialPrintLn(x) Serial.println(x)
#define SerialPrintBin(x) Serial.print(x, BIN)
#define SerialPrintLnBin(x) Serial.println(x, BIN)
#else
#define SerialPrint(x)
#define SerialPrintLn(x)
#define SerialPrintBin(x)
#define SerialPrintLnBin(x)
#endif

#define SDA_PORT PORTC
#define SDA_PIN 4
#define SCL_PORT PORTC
#define SCL_PIN 5

#define I2C_HARDWARE 1
#define I2C_TIMEOUT 100
#define I2C_FASTMODE 1
//#define I2C_SLOWMODE 1

#include <utils.hpp>
#define HTTP_REQ_BUF_SZ 200
#include <http_server.hpp>

SoftWire       sWire = SoftWire();
EthernetServer ethServer(80);
EthernetClient remoteClient;
PubSubClient   mqttClient(remoteClient);
OneWire        ds2401(A1);

#define ETH_SHIELD_RESET_PIN   A0

static const uint8_t NUM_IOS = 8;
static const uint8_t INPUT_HIGH_STATE = 0xFF;
static const uint8_t INPUT_LOW_STATE = 0x00;
static const uint8_t INPUT_PINS_START = 2;
static const uint8_t MCP27008_ADRESS = 0x27;
static const uint8_t PCF8591_ADRESS = 0x48;
static const uint8_t ATTINY_ADDRESS = 0x22;

static struct StoredSettings {
  MqttSettings mqtt;
  uint32_t     open_time[4];    // open time in miliseconds
  uint32_t     close_time[4];   // close time in miliseconds
  uint32_t     tilt_time[4];    // tilt time in miliseconds
  uint32_t     long_press_time; // Long press time in ms.
} boardSettings;

#define EEPROM_VERSION_OFFSET  0
#define EEPROM_SETTINGS_OFFSET 1
#define EEPROM_VERSION         0x53
#define EEPROM_POS_OFFSET ((EEPROM_SETTINGS_OFFSET)+sizeof(StoredSettings))
#define EEPROM_TILT_OFFSET (EEPROM_POS_OFFSET+4)

static const uint8_t TOPIC_ID_START_INDEX = 8;

static const uint8_t TOPIC_CMD_INDEX = 17;
char topicBase[TOPIC_CMD_INDEX+15] = { "COVERIO/\0\0\0\0\0\0\0\0/\0" };

#define CMD_TOPIC_CH_INDEX 4
const char* cmdSetSubtopic      = { "cmd/ /set\0" };
const char* cmdPosSubtopic      = { "cmd/ /pos\0" };
const char* cmdTiltSubtopic     = { "cmd/ /tilt\0" };
const char* cmdCalibSubtopic    = { "cmd/ /calib\0" };

#define STATE_TOPIC_CH_INDEX 6
const char* statePosSubtopic    = { "state/ /pos\0" };
const char* stateTiltSubtopic   = { "state/ /tilt\0" };
const char* subscribeSubtopic   = { "cmd/+/+\0" };

const char* getSubscribeTopic()
{
  strcpy(topicBase+TOPIC_CMD_INDEX, subscribeSubtopic);
  return topicBase;
}
const char* getCmdTopic(uint8_t chInd, const char* subTopic)
{
  strcpy(topicBase+TOPIC_CMD_INDEX, subTopic);
  topicBase[TOPIC_CMD_INDEX+CMD_TOPIC_CH_INDEX] = chInd + '1';
  return topicBase;
}
const char* getStateTopic(uint8_t chInd, const char* subTopic)
{
  strcpy(topicBase+TOPIC_CMD_INDEX, subTopic);
  topicBase[TOPIC_CMD_INDEX+STATE_TOPIC_CH_INDEX] = chInd + '1';
  return topicBase;
}

char clientId[]          = { "COVERIO_\0\0\0\0\0\0\0\0\0" };

// Update these with values suitable for your network.
byte mac[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static  uint8_t  inputCounters[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  byte     inputsState = 0;
//static  byte     inputsStateToPublish = 0;
static  uint32_t lastinputsReleased[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  uint32_t lastinputsPressed[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  byte     lastinputsState = 0;
static  uint32_t hwResetCount = 0;
static  volatile int8_t requestedPos[4] = { -1, -1, -1, -1 };
static  volatile int8_t requestedTilt[4] = { -1, -1, -1, -1 };
static  volatile bool   requestedCalibration[4] = { false, false, false, false };

#define PUBLISH_POS_FLAG  0x01
#define PUBLISH_TILT_FLAG 0x02
#define PUBLISH_ALL_FLAG  0x03
static  volatile byte publishFlag[4] = {0, 0, 0, 0};
/*
static const int8_t   MOVING_UP = -1;
static const int8_t   MOVING_DOWN = 1;
static const int8_t   MOVING_STOP = 1;
*/
static const int   MOVE_RANGE = 100;

#define BIT_UP(chInd)   ((chInd)*2+1)
#define BIT_DOWN(chInd) ((chInd)*2)

//static  int8_t outputsDir[] = { 0, 0, 0, 0 };
//static  int blindsPos[] = { 0, 0, 0, 0 };
//static  int blindsTiltPos[] = { 0, 0, 0, 0 };

//static  int8_t outputsExpectedPos[] = { 0, 0, 0, 0 };
//static  int8_t outputsExpectedTilt[] = { 0, 0, 0, 0 };

static  uint8_t outputsState = 0;
//static  byte    outputsStateToPublish = 0;

#define STATE_MACHINE_STEP_TIME 2

#define CHANNEL_ENTER_IDLE      0
#define CHANNEL_MONITOR_IDLE    1
#define CHANNEL_ENTER_POS       2
#define CHANNEL_MONITOR_POS     3
#define CHANNEL_ENTER_TILT      4
#define CHANNEL_MONITOR_TILT    5
#define CHANNEL_ENTER_CALIBRATE 6


static const byte ChannelDir_None = 0;
static const byte ChannelDir_Down = 1;
static const byte ChannelDir_Up = 2;


const char* stateNames[] = 
{
  "EI",
  "MI",
  "EP",
  "MP",
  "ET",
  "MT",
  "EC"
};

static int8_t    currentTilt[4];
static int8_t    currentPos[4];

static int8_t    expectedPos[4];
static int8_t    expectedTilt[4];

static uint16_t  tilt_time[4];
static uint16_t  move_time[4];
static uint32_t  stateEnterMs[4];

static int8_t    movePosDiff[4];
static int8_t    stateEnterPos[4];
static int8_t    stateEnterTilt[4];
static byte      currentState[4];

static byte      currentLevel[4];

static byte      calibrationStep[4] = {0, 0, 0, 0};
static uint16_t  calibrationCounter[4] = {0, 0, 0, 0};
static uint32_t  calibrationTimestamp[4] = {0, 0, 0, 0};
static byte      stopRequested[4] = { false, false, false, false};

static  byte     monitorCurrent = 0;
static uint32_t  currentLevelLastCheck = 0xFFFFF;

bool noCurrentFlow(uint8_t chInd)
{
  return currentLevel[chInd] < 30;
}
/*
 * 
 */
void startMonitorCurrentLevel(byte ch)
{
  monitorCurrent |= 1 << ch;
  currentLevelLastCheck = 0;
}
/*
 * 
 */
void stopMonitorCurrentLevel(byte ch)
{
  monitorCurrent &= ~(1 << ch);
}
/*
 * 
 */
void updateCurrentLevel()
{

  if(!monitorCurrent)
    return;
    
  const uint32_t millisNow = millis();

  if(calcTimestampDiff(currentLevelLastCheck, millisNow) < 50)
  {
    return;
  }
  currentLevelLastCheck = millisNow;
  
  sWire.requestFrom(ATTINY_ADDRESS, (uint8_t)4);
  while(sWire.available() < 4)
  {
  }
 
  for(byte byteNr = 0; byteNr < 4; ++byteNr)
  {
    currentLevel[byteNr] = (byte)sWire.read();
    SerialPrint(currentLevel[byteNr])SerialPrint(", ");
  }
  SerialPrintLn("");
}
/*
 * 
 */
void printStats(byte i)
{
#ifdef COVERIO_DEBUG_MODE
  static uint16_t counter[4];
  ++counter[i];
  if(counter[i] < 256)
    return;
  counter[i] = 0;

  //for(byte i = 0; i < 4; ++i)
  {
    Serial.print("CH:");
    Serial.print(i);
    Serial.print(F(",C:"));
    Serial.print(currentLevel[i]);
    Serial.print(F(",P:"));
    Serial.print(currentPos[i]);
    Serial.print(F(",T:"));
    Serial.println(currentTilt[i]);
  }
#endif
}
/*
 * 
 */
void setChannelPublishFlag(uint8_t chInd, byte flag)
{
  publishFlag[chInd] |= flag;
}
/*
 * 
 */
void setChannelMoveDir(uint8_t chInd, byte dir)
{
  byte expectedtState = outputsState;

  expectedtState = (expectedtState & ~(0x03 << (chInd*2))) | (dir << (chInd*2));

  SerialPrint(F("O:"));
  SerialPrintLnBin(expectedtState);
  if(outputsState != expectedtState)
  {
    outputsState = expectedtState;
    //set_bit(outputsStateToPublish, index);
    MCP27008_write(sWire, MCP27008_ADRESS, outputsState);
  }
}
byte getChannelMoveDir(uint8_t chInd)
{
  return ((outputsState >> (chInd*2)) & 0x03);
}
/*
 * 
 */
bool upButtonPressed(uint8_t chInd)
{
    // Check if button was presed
  if(get_bit(inputsState, BIT_UP(chInd)) && 
        get_bit(inputsState, BIT_UP(chInd)) != get_bit(lastinputsState, BIT_UP(chInd)))
  {
    return true;
  }
  return false;
}
/*
 * 
 */
bool downButtonPressed(uint8_t chInd)
{
    // Check if button was presed
  if(get_bit(inputsState, BIT_DOWN(chInd)) &&
      get_bit(inputsState, BIT_DOWN(chInd)) != get_bit(lastinputsState, BIT_DOWN(chInd)))
  {
     return true;
  }
  return false;
}
/*
 * 
 */
bool upButtonReleased(uint8_t chInd)
{
    // Check if any button was presed
  if(!get_bit(inputsState, BIT_UP(chInd)) && 
        get_bit(inputsState, BIT_UP(chInd)) != get_bit(lastinputsState, BIT_UP(chInd)))
  {
    return true;
  }
  return false;
}
/*
 * 
 */
bool downButtonReleased(uint8_t chInd)
{
    // Check if any button was presed
  if(!get_bit(inputsState, BIT_DOWN(chInd)) && 
        get_bit(inputsState, BIT_DOWN(chInd)) != get_bit(lastinputsState, BIT_DOWN(chInd)))
  {
    return true;
  }
  return false;
}
/*
 * 
 */
bool calibPressed(uint8_t chInd)
{
  // Check if button is presed
  if(!get_bit(inputsState, BIT_DOWN(chInd)) || !get_bit(inputsState, BIT_UP(chInd)))
  {
     return false;
  }
  const uint32_t millisNow = millis();

  if(calcTimestampDiff(lastinputsPressed[BIT_UP(chInd)], millisNow) >= 2000
      && calcTimestampDiff(lastinputsPressed[BIT_DOWN(chInd)], millisNow) >= 2000)
  {
     return true;
  }
  return false;
}
/*
 * 
 */
bool upButtonLongPressed(uint8_t chInd)
{
  // Check if button is presed
  if(!get_bit(inputsState, BIT_UP(chInd)) || get_bit(inputsState, BIT_DOWN(chInd)))
  {
     return false;
  }
  const uint32_t millisNow = millis();

  if(calcTimestampDiff(lastinputsPressed[BIT_UP(chInd)], millisNow) >= boardSettings.long_press_time)
  {
     return true;
  }
  return false;
}
/*
 * 
 */
bool downButtonLongPressed(uint8_t chInd)
{
  // Check if button is presed
  if(!get_bit(inputsState, BIT_DOWN(chInd)) || get_bit(inputsState, BIT_UP(chInd)))
  {
     return false;
  }
  const uint32_t millisNow = millis();

  if(calcTimestampDiff(lastinputsPressed[BIT_DOWN(chInd)], millisNow) < boardSettings.long_press_time)
  {
     return false;
  }
  return true;
}
/*
 * 
 */
void enterState(byte chInd, byte expectedState)
{
    if(currentState[chInd] == expectedState)
    {
      return;
    }
    SerialPrint(F("CH:"));
    SerialPrint(chInd);
    SerialPrint(F(" NS:"));
    SerialPrintLn(stateNames[expectedState]);
    
    stateEnterPos[chInd] = currentPos[chInd];
    stateEnterTilt[chInd] = currentTilt[chInd];
    stateEnterMs[chInd] = millis();
    currentState[chInd] = expectedState;
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
void idleEnter(uint8_t chInd)
{
  SerialPrint(F("CH:")); SerialPrint(chInd);
  SerialPrint(F(",P:")); SerialPrint(currentPos[chInd]);
  SerialPrint(F(",T:")); SerialPrintLn(currentTilt[chInd]);
  
  setChannelMoveDir(chInd, ChannelDir_None);
  setChannelPublishFlag(chInd, PUBLISH_ALL_FLAG);
  stopRequested[chInd] = false;
  stopMonitorCurrentLevel(chInd);
  //requestedPos[chInd] = 0xFF;
  //requestedTilt[chInd] = 0xFF;
  //requestedCalibration[chInd] = false;

  EEPROM.write(EEPROM_POS_OFFSET+chInd, currentPos[chInd]);
  EEPROM.write(EEPROM_TILT_OFFSET+chInd, currentTilt[chInd]);
  
  enterState(chInd, CHANNEL_MONITOR_IDLE);
}
/*
 * Monitorowanie stanu bezczynności
 */
void idleMonitor(uint8_t chInd)
{
  // Do nothing if we are not at least 1000ms in idle.
  if(calcTimestampDiff(stateEnterMs[chInd], millis()) < 300)
  {
    return;
  }
  if(requestedCalibration[chInd] || calibPressed(chInd))
  {
    calibrationStep[chInd] = 0;
    requestedCalibration[chInd] = false;
    SerialPrint(F("B cal:")); SerialPrintLn(chInd); 
    enterState(chInd, CHANNEL_ENTER_CALIBRATE);
  }
  else if(requestedPos[chInd] >= 0 && requestedPos[chInd] <= 100)
  {
    //Serial.print(F("Pos:")); Serial.println(requestedPos[chInd]);
    expectedPos[chInd] = requestedPos[chInd];
    requestedPos[chInd] = -1;
    enterState(chInd, CHANNEL_ENTER_POS);
  }
  else if(requestedTilt[chInd] >= 0 && requestedTilt[chInd] <= 100)
  {
    //Serial.print(F("Tilt:")); Serial.println(requestedTilt[chInd]);
    expectedTilt[chInd] = requestedTilt[chInd];
    requestedTilt[chInd] = -1;
    enterState(chInd, CHANNEL_ENTER_TILT); 
  }
  else if(upButtonLongPressed(chInd))
  {
    SerialPrint(F("Bt up:")); SerialPrintLn(chInd);
    expectedPos[chInd] = 0;
    enterState(chInd, CHANNEL_ENTER_POS);
  }
  else if(downButtonLongPressed(chInd))
  {
    SerialPrint(F("Bt down:")); SerialPrintLn(chInd);
    expectedPos[chInd] = 100;
    enterState(chInd, CHANNEL_ENTER_POS); 
  }
  else if(upButtonReleased(chInd))
  {
    SerialPrint(F("Bt Tup:")); SerialPrintLn(chInd);
    expectedTilt[chInd] = currentTilt[chInd] - 20;
    if(expectedTilt[chInd] > 100)
      expectedTilt[chInd] = 100;
    enterState(chInd, CHANNEL_ENTER_TILT); 
  }
  else if(downButtonReleased(chInd))
  {
    SerialPrintLn(F("Bt Tdown:")); SerialPrintLn(chInd);
    expectedTilt[chInd] = currentTilt[chInd] + 20;
    if(expectedTilt[chInd] < 0)
      expectedTilt[chInd] = 0;
    enterState(chInd, CHANNEL_ENTER_TILT); 
  }
}
/*
 * Wejscie w stan zmiany pozycji
 */
void posEnter(uint8_t chInd)
{
  int8_t posDiff = movePosDiff[chInd] = currentPos[chInd] - expectedPos[chInd];
  int8_t tiltDiff;

  if(expectedPos[chInd] < 0 || expectedPos[chInd] > 100 || posDiff == 0)
  {
    enterState(chInd, CHANNEL_ENTER_IDLE);
    return;
  }
  if(posDiff > 0)
  {
    tiltDiff = currentTilt[chInd];
    move_time[chInd] = ((uint32_t)boardSettings.open_time[chInd] * (uint32_t)abs(posDiff)) / (uint32_t)100;
    
    requestedTilt[chInd] = currentTilt[chInd];
    currentTilt[chInd] = 0;
    setChannelMoveDir(chInd, ChannelDir_Up);
  }
  else
  {
    tiltDiff = 100 - currentTilt[chInd];
    move_time[chInd] = ((uint32_t)boardSettings.close_time[chInd] * (uint32_t)abs(posDiff)) / (uint32_t)100;

    requestedTilt[chInd] = currentTilt[chInd];
    currentTilt[chInd] = 100;
    setChannelMoveDir(chInd, ChannelDir_Down);
  }
  
  move_time[chInd] += ((uint32_t)boardSettings.tilt_time[chInd] * 
                        (uint32_t)abs(tiltDiff)) / (uint32_t)100;
    
  setChannelPublishFlag(chInd, PUBLISH_TILT_FLAG);
  startMonitorCurrentLevel(chInd);
  //Serial.print(F("Mt:")); Serial.println(move_time[chInd]);
  //Serial.print(F("Pos: ")); Serial.println(stateEnterPos[chInd]);
  enterState(chInd, CHANNEL_MONITOR_POS);
}
/*
 * Monitorowanie stanu zmiany pozycji
 */
void posMonitor(uint8_t chInd)
{
  if(upButtonPressed(chInd) || downButtonPressed(chInd) || stopRequested[chInd])
  {
    //Serial.println(F("FS"));
    enterState(chInd, CHANNEL_ENTER_IDLE);
    return;
  }
  int32_t stateTime = calcTimestampDiff(stateEnterMs[chInd], millis());
  bool    noCurrent = stateTime > 200 && noCurrentFlow(chInd);
  bool    timeout = stateTime >= move_time[chInd];
  if((timeout && expectedPos[chInd] != 100 && expectedPos[chInd] != 0) || noCurrent)
  {
    if(noCurrent)
    { 
      if(ChannelDir_Down == getChannelMoveDir(chInd))
      {
        currentPos[chInd] = 100;
        currentTilt[chInd] = 100;
        //Serial.println(F("PM:D"));
      }
      else
      {
        currentPos[chInd] = 0;
        currentTilt[chInd] = 0;
        //Serial.println(F("PM:U"));
      }
    }
    else
    {
      currentPos[chInd] = expectedPos[chInd];
      //Serial.println(F("PM:P"));
    }
    move_time[chInd] = 0;
    enterState(chInd, CHANNEL_ENTER_IDLE);
  }
  else
  {
    currentPos[chInd] = stateEnterPos[chInd] + (expectedPos[chInd] - (int32_t)stateEnterPos[chInd]) * 
                          (stateTime*100u / move_time[chInd]) / 100u;
    if(currentPos[chInd] > 100)
      currentPos[chInd] = 100;
    else if(currentPos[chInd] < 0)
      currentPos[chInd] = 0;
  }
  setChannelPublishFlag(chInd, PUBLISH_POS_FLAG);
  printStats(chInd);
}
/*
 * Wejscie do stanu zmiany kąta lameli
 */
void tiltEnter(uint8_t chInd)
{
  int8_t tiltDiff = currentTilt[chInd] - expectedTilt[chInd];

  if(expectedTilt[chInd] < 0 || expectedTilt[chInd] > 100 || tiltDiff == 0)
  {
    enterState(chInd, CHANNEL_ENTER_IDLE);
    return;
  }
  
  tilt_time[chInd] = ((uint32_t)boardSettings.tilt_time[chInd] * 
                      (uint32_t)abs(tiltDiff)) / (uint32_t)100;
    
  if(tiltDiff > 0)
  {
    setChannelMoveDir(chInd, ChannelDir_Up);
  }
  else
  {
    setChannelMoveDir(chInd, ChannelDir_Down);
  }
  setChannelPublishFlag(chInd, PUBLISH_TILT_FLAG);
  startMonitorCurrentLevel(chInd);
  //Serial.print(F("Tt:")); Serial.println(tilt_time[chInd]);
  enterState(chInd, CHANNEL_MONITOR_TILT);
}
/*
 * Monitorowanie stanu zmiany kąta lameli
 */
void tiltMonitor(uint8_t chInd)
{
  if(upButtonPressed(chInd) || downButtonPressed(chInd) || stopRequested[chInd])
  {
    SerialPrintLn(F("FS"));
    enterState(chInd, CHANNEL_ENTER_IDLE);
    return;
  }
  int32_t stateTime = calcTimestampDiff(stateEnterMs[chInd], millis());
  bool    noCurrent = stateTime > 200 && noCurrentFlow(chInd);
  if(stateTime >= tilt_time[chInd] || noCurrent)
  {
    if(noCurrent)
    { 
      if(ChannelDir_Down == getChannelMoveDir(chInd))
      {
        currentPos[chInd] = 100;
        currentTilt[chInd] = 100;
      }
      else
      {
        currentPos[chInd] = 0;
        currentTilt[chInd] = 0;
      }
    }
    else
    {
      currentTilt[chInd] = expectedTilt[chInd];
    }
    tilt_time[chInd] = 0;
    enterState(chInd, CHANNEL_ENTER_IDLE);
  }
  else
  {
    currentTilt[chInd] = stateEnterTilt[chInd] + (expectedTilt[chInd] - stateEnterTilt[chInd]) * 
                           (stateTime*100 / tilt_time[chInd]) / 100;
    if(currentTilt[chInd] > 100)
      currentTilt[chInd] = 100;
    else if(currentTilt[chInd] < 0)
      currentTilt[chInd] = 0;
  }
  setChannelPublishFlag(chInd, PUBLISH_TILT_FLAG);
  printStats(chInd);
}
/*
 * 
 */
void calibrateEnter(uint8_t chInd)
{
  const uint32_t millisNow = millis();

  switch(calibrationStep[chInd])
  {
    case 0: // Start current monitoring, reset fields, start moving to 0 pos.
      startMonitorCurrentLevel(chInd);
      setChannelMoveDir(chInd, ChannelDir_Up);
      calibrationCounter[chInd] = 0;
      calibrationStep[chInd] = 1;
      SerialPrintLn(F("0-1"));
      break;
    case 1: // Wait 200ms
      if(++calibrationCounter[chInd] >= 100)
      {
        calibrationStep[chInd] = 2;
        SerialPrintLn(F("1-2"));
      }
    break;
    case 2: // wait while cover is moving
      if(noCurrentFlow(chInd))
      {
        setChannelMoveDir(chInd, ChannelDir_None);
        currentTilt[chInd] = 0;
        currentPos[chInd] = 0;
        calibrationCounter[chInd] = 0;
        calibrationStep[chInd] = 3;
        SerialPrintLn(F("2-3"));
      }
    break;
    case 3:
      if(++calibrationCounter[chInd] > 1000) //Go to next step after 1s
      {
        calibrationStep[chInd] = 4;
        SerialPrintLn(F("3-4"));
      }
    break;
    case 4: // Start moving down
      setChannelMoveDir(chInd, ChannelDir_Down);
      calibrationTimestamp[chInd] = millisNow;
      calibrationCounter[chInd] = 0;
      calibrationStep[chInd] = 5;
      SerialPrintLn(F("4-5"));
    break;
    case 5: // Wait 200ms, measure current to check if moving
      if(++calibrationCounter[chInd] >= 100)
      {
        calibrationStep[chInd] = 6;
        SerialPrintLn(F("5-6"));
      }
    break;
    case 6: // Wait while still moving
      if(noCurrentFlow(chInd))
      {
        boardSettings.close_time[chInd] = calcTimestampDiff(calibrationTimestamp[chInd], millisNow);
        
        setChannelMoveDir(chInd, ChannelDir_None);
        
        currentTilt[chInd] = 100;
        currentPos[chInd] = 100;
        calibrationCounter[chInd] = 0; 
        calibrationStep[chInd] = 7;
        SerialPrint(F("6-7:Ct:")); SerialPrintLn(boardSettings.close_time[chInd]);
      }
    break;
    case 7: // wait 1s, start moving up.
      if(++calibrationCounter[chInd] >= 1000)
      {
        setChannelMoveDir(chInd, ChannelDir_Up);
        calibrationTimestamp[chInd] = millisNow;
        calibrationCounter[chInd] = 0;
        calibrationStep[chInd] = 8;
        SerialPrint(F("7-8"));
      }
    break;
    case 8:  // Wait 200ms, measure current to check if moving
      if(++calibrationCounter[chInd] >= 100)
      {
        calibrationStep[chInd] = 9;
        SerialPrintLn(F("8-9"));
      }
    break;
    case 9: // Wait while still moving
      if(noCurrentFlow(chInd))
      {
        boardSettings.open_time[chInd] = calcTimestampDiff(calibrationTimestamp[chInd], millisNow);

        EEPROM.put(EEPROM_SETTINGS_OFFSET, boardSettings);

        currentTilt[chInd] = 0;
        currentPos[chInd] = 0;
        calibrationStep[chInd] = 15;
        Serial.println(F("EEPROM updated")); 
        SerialPrint(F("9-15:Ot:")); SerialPrintLn(boardSettings.open_time[chInd]);
      }
    break;
    case 15:
        enterState(chInd, CHANNEL_ENTER_IDLE);
        calibrationStep[chInd] = 0;
        SerialPrintLn(F("15-I"));
    break;
    default:
        SerialPrintLn(F("Wrg calib"));
        enterState(chInd, CHANNEL_ENTER_IDLE);
        calibrationStep[chInd] = 0;
    break;
  }
}
/*
 * 
 */
void executeStateMachine()
{
  updateCurrentLevel();
  for(byte chInd = 0; chInd < 4; ++chInd)
  {
    switch(currentState[chInd])
    {
      case CHANNEL_ENTER_IDLE:
        idleEnter(chInd);
      break;
      case CHANNEL_MONITOR_IDLE:
        idleMonitor(chInd);
      break;
      case CHANNEL_ENTER_POS:
        posEnter(chInd);
      break;
      case CHANNEL_MONITOR_POS:
        posMonitor(chInd);
      break;
      case CHANNEL_ENTER_TILT:
        tiltEnter(chInd);
      break;
      case CHANNEL_MONITOR_TILT:
        tiltMonitor(chInd);
      break;
      case CHANNEL_ENTER_CALIBRATE:
        calibrateEnter(chInd);
      break;
      default:
        //Serial.println(F("Wrg state"));
      break;
    }
  }
}

void readInputsInterruptHandler()
{
  const uint32_t millisNow = millis();
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
      set_bit(inputsState , i);
    }
    else if(inputCounters[i] == INPUT_LOW_STATE)
    {
      clear_bit(inputsState, i);
    }
    if(get_bit(inputsState, i) != get_bit(lastinputsState, i))
    {
      if(get_bit(inputsState, i))
      {
        lastinputsPressed[i] = millisNow;
      }
      else
      {
        lastinputsReleased[i] = millisNow;
      }
    }
  }

  executeStateMachine();
  
  lastinputsState = inputsState;
}

void callback(char* topic, byte* payload, unsigned int length)
{
  int8_t channelInd = 0;
  int8_t value;
  char* substr;
  SerialPrintLn(F("Rcv: "));
  SerialPrintLn(topic);
  SerialPrint("[");
  int i=0;
  for (i=0;i<length;i++) {
    SerialPrint((char)payload[i]);
  }
  SerialPrintLn("]");

  if(length == 0)
  {
    for(byte i=0; i<4; ++i)
    {
      setChannelPublishFlag(i, PUBLISH_ALL_FLAG);
    }
    return;
  }
  
  substr = strstr(topic, "/set");
  if(substr)
  {
    channelInd = *(substr-1) - '1';
    Serial.print(channelInd);
    if(length == 4 && (memcmp(payload, "STOP", 4) == 0 || memcmp(payload, "stop", 4) == 0))
    {
      stopRequested[channelInd] = true;
      SerialPrintLn(": Stop");
    }
    else if(length == 4 && (memcmp(payload, "OPEN", 4) == 0 || memcmp(payload, "open", 4) == 0))
    {
      requestedPos[channelInd] = 0;
      
      SerialPrintLn(": Pos = 0");
      //requestedTilt[channelInd] = 0;
    }
    else if(length == 5 && (memcmp(payload, "CLOSE", 5) == 0 || memcmp(payload, "close", 5) == 0))
    {
      requestedPos[channelInd] = 100;
      //requestedTilt[channelInd] = 100;
      SerialPrintLn(": Pos = 100");
    }
    return;
  }
  substr = strstr(topic, "/pos");
  if(substr)
  {
    channelInd = *(substr-1) - '1';
    value = atoi((const char*)payload);
    requestedPos[channelInd] = value;
    
    SerialPrint("Pos: ");SerialPrintLn(requestedPos[channelInd]);
    return;
  }
  substr = strstr(topic, "/tilt");
  if(substr)
  {
    channelInd = *(substr-1) - '1';
    value = atoi((const char*)payload);
    requestedTilt[channelInd] = value;
    SerialPrint("Tilt: ");SerialPrintLn(requestedTilt[channelInd]);
    return;
  }
  substr = strstr(topic, "/calib");
  if(substr)
  {
    channelInd = *(substr-1) - '1';
    requestedCalibration[channelInd] = true;
    SerialPrintLn("Calib");
    return;
  }
}
/*
 * 
 */
void publishMsg(PubSubClient& client, const char* topic, const char* payload)
{
  SerialPrintLn(F("Pub: "));
  SerialPrintLn(topic);
  SerialPrintLn(F(": "));
  SerialPrintLn(payload);

  client.publish((const char*)topic, payload, true);
}
/*
 * 
 */
void checkFlagsAndPublish(PubSubClient& client)
{
  const uint32_t millisNow = millis();
  static uint32_t lastCheck = 0;
  if(calcTimestampDiff(lastCheck, millisNow) < 500)
  {
    return;
  }
  lastCheck = millisNow;
  
  char buffer[5];
  byte currentPublishFlag;

  for(byte chInd = 0; chInd < 4; ++chInd)
  {
    noInterrupts();
    currentPublishFlag = publishFlag[chInd];
    publishFlag[chInd] = 0;
    interrupts();
  
    if(currentPublishFlag == 0)
      continue;
      
    if(currentPublishFlag & (PUBLISH_POS_FLAG << chInd) != 0)
    {
      itoa((int)currentPos[chInd], buffer, 10);
      publishMsg(client, getStateTopic(chInd, statePosSubtopic), buffer);
    }
      
    if(currentPublishFlag & (PUBLISH_TILT_FLAG << chInd) != 0)
    {
      itoa((int)currentTilt[chInd], buffer, 10);
      publishMsg(client, getStateTopic(chInd, stateTiltSubtopic), buffer);
    }
  }
}
/*
 * 
 */
/*void checkInputsAndPublish(PubSubClient& client)
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
    if(get_bit(boardSettings.mode, i) && get_bit(currentInputsStateToPublish, i))
    {
      inputStateTopic[TOPIC_IN_STATE_CHANNEL_INDEX] = i + '1';
      SerialPrintLn(inputStateTopic);
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
}*/
/*
 * 
 */
void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
#ifdef COVERIO_DEBUG_MODE
  Serial.print(F("COVERIO DEBUG ver: "));
#else
  Serial.print(F("COVERIO ver: "));
#endif
  Serial.println(SOFT_VER);
 
  for(byte i = 0; i < NUM_IOS; ++i)
  {
    pinMode(INPUT_PINS_START + i, INPUT_PULLUP);      // sets the switch sensor digital pin as input
  }
  
  pinMode(ETH_SHIELD_RESET_PIN, OUTPUT);

  if(EEPROM.read(EEPROM_VERSION_OFFSET) != EEPROM_VERSION)
  {
    SerialPrintLn(F("Clearing EEPROM!"));
    memset(&boardSettings, 0, sizeof(boardSettings));
  
    for(uint8_t i = 0; i < 4; ++i)
    {
      boardSettings.open_time[i] = 0;
      boardSettings.close_time[i] = 0;
      boardSettings.tilt_time[i] = 0;
    }
    boardSettings.long_press_time  = 2500;
    EEPROM.write(EEPROM_VERSION_OFFSET, EEPROM_VERSION);
    EEPROM.put(EEPROM_SETTINGS_OFFSET, boardSettings);
  }
  
  EEPROM.get(EEPROM_SETTINGS_OFFSET, boardSettings);

  for(byte i = 0; i < 4; ++i)
  {
    currentPos[i] = EEPROM.read(EEPROM_POS_OFFSET+i);
    currentTilt[i] = EEPROM.read(EEPROM_TILT_OFFSET+i);
  }
  
  getMacAddress(ds2401, mac);
    
  for (byte i = 0; i < 4; i++)
  {
    byteToHexStr(mac[i+2], clientId + (TOPIC_ID_START_INDEX + i*2));
  }
  memcpy(topicBase+TOPIC_ID_START_INDEX, clientId+TOPIC_ID_START_INDEX, 8);

  Serial.println(clientId);
  Serial.println(topicBase);
  
#ifdef COVERIO_DEBUG_MODE
  for(byte i = 0; i < 4; ++i)
  {
    Serial.print(boardSettings.mqtt.mqtt_ip[i]);
    if(i < 3)Serial.print(".");
  }
  Serial.print(F(":"));
  Serial.println(boardSettings.mqtt.mqtt_port);
  
  Serial.print(boardSettings.mqtt.mqtt_username);
  Serial.print(F(":"));
  Serial.println(boardSettings.mqtt.mqtt_password);
#endif

  sWire.begin();
/*
  scanI2C(sWire);
*/
  MCP27008_setup(sWire, MCP27008_ADRESS);
  MCP27008_write(sWire, MCP27008_ADRESS, outputsState);

  mqttClient.setCallback(callback);

  MsTimer2::set(2, readInputsInterruptHandler);
  MsTimer2::start();
}

static uint32_t lastConnectMillis = 0;
static byte mqttConnectionFlag;
void handleMqttClient()
{
  if (!mqttClient.connected())
  {
    const uint32_t millisNow = millis();
    if(mqttConnectionFlag == 0 || calcTimestampDiff(lastConnectMillis, millisNow) >= 20000)
    {
      mqttConnectionFlag = 1;
      lastConnectMillis = millisNow;
      
      mqttClient.setServer(boardSettings.mqtt.mqtt_ip, boardSettings.mqtt.mqtt_port);
      Serial.print(("MQTT connecting..."));
      if (mqttClient.connect(clientId, boardSettings.mqtt.mqtt_username, boardSettings.mqtt.mqtt_password))
      {
        const char* subTopic = getSubscribeTopic();
        Serial.println(subTopic);
        mqttClient.subscribe(subTopic);
        mqttClient.subscribe("status");
        for(byte chInd = 0; chInd < 4; ++chInd)
        {
          setChannelPublishFlag(chInd, PUBLISH_ALL_FLAG);
        }
      }
      else
      {
        Serial.print((" err= "));
        Serial.println(mqttClient.state());
      }
    }
  }
  else
  {
    //checkOutputsAndPublish(mqttClient);
    //checkInputsAndPublish(mqttClient);
    checkFlagsAndPublish(mqttClient);
    mqttClient.loop();
  }
}

CustomHandlers customHandlers = 
{
  .customProcess = processCustomParams,
  .customForms = addCustomForms,
  .mqtt_ptr = &boardSettings.mqtt
};
/*
 * 
 */
/*bool parseOpenTime(const char* reqStr, uint32_t* timersArray)
{
  int  iArray[4];
  bool result = false;
  char* strPtr = strstr(reqStr, "&open_time=");
  if(strPtr)
  {
    strPtr += 10;
    if(sscanf(strPtr, "=%d,%d,%d,%d&", iArray, iArray+1, iArray+2, iArray+3) == 4)
    {
      for(byte i = 0; i < 4; ++i)
        timersArray[i] = iArray[i];
        
      result = true;
    }
  }
  return result;
}*/
/*
 * 
 */
/*bool parseCloseTime(const char* reqStr, uint32_t* timersArray)
{
  int  iArray[4];
  bool result = false;
  char* strPtr = strstr(reqStr, "&close_time=");
  if(strPtr)
  {
    strPtr += 11;
    if(sscanf(strPtr, "=%d,%d,%d,%d&", iArray, iArray+1, iArray+2, iArray+3) == 4)
    {
      for(byte i = 0; i < 4; ++i)
        timersArray[i] = iArray[i];
        
      result = true;
    }
  }
  return result;
}*/
/*
 * 
 */
bool parseTiltTime(const char* reqStr, uint32_t* timersArray)
{
  int  iArray;
  bool result = false;
  char formName[] = { "_t =\0" };
  for(byte i = 0; i < 4; ++i)
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
/*
 * 
 */
bool parseCalibration(const char* reqStr, volatile bool * flagArray)
{
  int  iArray;
  bool result = false;
  char formName[] = { "_c =\0" };
  for(byte i = 0; i < 4; ++i)
  {
    formName[2] =  i + '1';
    char* strPtr = strstr(reqStr, formName);
    if(strPtr)
    {
      strPtr += 3;
      if(*(strPtr+1) == '1')
      {
        flagArray[i] = true;
      }
      else
      {
        flagArray[i] = false;
      }
      result = true;
    }
  }
  return result;
}
/*
 * 
 */
bool processCustomParams(const char* reqStr)
{
  bool result = false;
  bool runCalib = false;
  /*if(parseOpenTime(reqStr, boardSettings.open_time))
  {
    Serial.print(F("Received: open_time\t:"));
    for(byte i = 0; i < 4; ++i)
    {
      Serial.print(boardSettings.open_time[i]);
      if(i<3)Serial.print(F(","));
    }
    Serial.println(F(""));
    result = true;
  }
  if(parseCloseTime(reqStr, boardSettings.close_time))
  {
    Serial.print(F("Received: close_time\t:"));
    for(byte i = 0; i < 4; ++i)
    {
      Serial.print(boardSettings.close_time[i]);
      if(i<3)Serial.print(F(","));
    }
    Serial.println(F(""));
    result = true;
  }*/
  if(parseTiltTime(reqStr, boardSettings.tilt_time))
  {
    SerialPrint(F("R: tilt\t:"));
    for(byte i = 0; i < 4; ++i)
    {
      SerialPrint(boardSettings.tilt_time[i]);
      if(i<3)SerialPrint(F(","));
      else SerialPrintLn(F(""));
    }
    result = true;
  }
  if(parseCalibration(reqStr, requestedCalibration))
  {
    Serial.print(F("R: calib:"));
    for(byte i = 0; i < 4; ++i)
    {
      SerialPrint(requestedCalibration[i]);
      if(i<3)SerialPrint(F(","));
      else SerialPrintLn(F(""));
    }
  }
  return result;
}
/*
 * 
 */
void addCustomForms(EthernetClient& client)
{
  client.println(F("\n\t\tOpen time(ms)\tClose time(ms)\tTilt time(ms)\t Calibrate"));
  for(byte i = 0; i < 4; ++i)
  {
    client.print(F("  Channel "));client.print(i+1);
    client.print(F("\t"));
    client.print(boardSettings.open_time[i]);
    client.print(F("\t\t"));
    client.print(boardSettings.close_time[i]);
    //client.print(F("\">"));
    client.print(F("\t\t<input type=\"text\" name=\"_t"));client.print(i+1); 
    client.print(F("\" maxlength=8 size=8 value=\"")); client.print(boardSettings.tilt_time[i]);
    client.print(F("\">"));
    client.print(F("\t<input type=\"checkbox\" name=\"_c")); client.print(i+1);
    client.println(F("\" value=\"1\">"));
  }

  client.println(F(""));
  client.print(F("<H1>State:</H1>"));
  client.print(F("\tMQTT client id:\t\t"));
  client.println(clientId);
  client.print(F("\tMQTT subscription:\t"));
  client.println(getSubscribeTopic());
  //client.print(F("\n\t\t\t\t"));
  //client.print(tiltCommandTopic);
  //client.print(F("\n\t\t\t\t"));
  //client.println(commandTopic);
  client.print(F("\tMQTT connection state:\t"));
  client.println(mqttClient.state());
  client.print(F("\tUptime:\t\t\t"));
  client.println(millis()); 
  client.print(F("\tVersion:\t\t"));
  client.println(SOFT_VER);
  client.print(F("\tEthernet resets:\t"));
  client.println(hwResetCount);
}
void addCustomSend(EthernetClient& client)
{
  byte i;
  /*client.println(F("strText+=\"&open_time=\";"));
  for(i = 0; i < 4; ++i)
  {
    client.print(F("strText+=document.getElementById(\"txt_form\").fo"));client.print(i);client.print(F(".value;"));
    if(i < 3)client.print(F("strText+=\",\";"));
  }
  client.println(F("strText+=\"&close_time=\";"));
  for(i = 0; i < 4; ++i)
  {
    client.print(F("strText+=document.getElementById(\"txt_form\").fc"));client.print(i);client.print(F(".value;"));
    if(i < 3)client.print(F("strText+=\",\";"));
  }
  client.println(F("strText+=\"&tilt_time=\";"));
  for(i = 0; i < 4; ++i)
  {
    client.print(F("strText+=document.getElementById(\"txt_form\").ft"));client.print(i);client.print(F(".value;"));
    if(i < 3)client.print(F("strText+=\",\";"));
  }
  client.println(F("strText+=\"&calib=\";"));
  client.print(F("strText+=document.getElementById(\"txt_form\").cc.checked;"));
  */
}
  
static uint32_t maintainLastMillis = 0, modCheckLastMillis = 0;
static byte maintainRes, connectionFlag;
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
  const uint32_t millisNow = millis();
  if(calcTimestampDiff(maintainLastMillis, millisNow) >= 10000)
  {
    maintainLastMillis = millisNow;
    
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
        Serial.println(F("Ethernet: none"));
      }
      connectionFlag = 0;
      return;
    }
  }
  
  handleMqttClient();
  
#ifndef COVERIO_DEBUG_MODE
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
#endif

}
