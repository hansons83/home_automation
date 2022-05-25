#include <Arduino.h>
#include <Wire.h>
//#include <TinyWireS.h>

const byte LED_PIN = 8;
const byte SLAVE_ADDR = 0x22 ;
const byte NUM_BYTES = 4;

int16_t  minVal[NUM_BYTES] = {512, 512, 512, 512};
int16_t  maxVal[NUM_BYTES] = {512, 512, 512, 512};
int16_t  diff[NUM_BYTES] = {0, 0, 0, 0};
byte analogPins[NUM_BYTES] = { A3, A2, A1, A0 };

byte ledState = LOW;
uint32_t lastLedMs = 0;

void requestISR()
{
    for (byte i=0; i < NUM_BYTES; i++) {
        Wire.write(diff[i] > 255 ? 255 : diff[i]);
    }
    ledState = ledState == LOW ? HIGH : LOW;
    digitalWrite(LED_PIN, ledState);
    lastLedMs = millis();
}

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  Wire.begin(SLAVE_ADDR);
  Wire.onRequest(requestISR);
}

uint32_t lastAnalogCheckMs = 0, lastCalcMs = 0;

void loop() {
  // put your main code here, to run repeatedly:
  //if(millis() - lastAnalogCheckMs > 2)
  {
    lastAnalogCheckMs = millis();
    for(byte i = 0; i < 4; ++i)
    {
      int aval = analogRead(analogPins[i]);
      if(aval > maxVal[i])
        maxVal[i] = aval;
      if(aval < minVal[i])
        minVal[i] = aval;
    }
  }
  if(millis() - lastCalcMs > 20)
  {
    lastCalcMs = millis();
    
    for(byte i = 0; i < 4; ++i)
    {
      diff[i] = maxVal[i] - minVal[i];
      maxVal[i] = minVal[i] = 512;
    }
  }

  if(millis() - lastLedMs > 5000)
  {
    lastLedMs = millis();
    
    ledState = ledState == LOW ? HIGH : LOW;
    digitalWrite(LED_PIN, ledState);
  }
}
