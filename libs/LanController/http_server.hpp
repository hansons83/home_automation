#pragma once
#include <Arduino.h>
#include <Ethernet2.h>


struct MqttSettings
{
  uint8_t  mqtt_ip[4];
  uint16_t mqtt_port;
  char     mqtt_username[32];
  char     mqtt_password[16];
};

enum HttpResult
{
	HTTP_NO_ACTION,
	HTTP_MQTT_CHANGE = 1,
	HTTP_USER_CHANGE = 2,
	HTTP_ALL_CHANGE  = 3
};

typedef bool (*httpRequestHandler)(char* data, uint16_t size);
typedef void (*httpResponseBuilder)(EthernetClient& client);

HttpResult httpHandle(EthernetServer& ethServer, MqttSettings& settings, httpRequestHandler regHandler, httpResponseBuilder respBuilder)
{
  //http://192.168.1.11/?ip=192.168.1.3&port=1883&user=openhabian&pwd=swiatek123&time=500&
  
  static const int srvBufferSize = 100;
  static char srvBuffer[150];
  static byte srvBufferPos = 0;
  static EthernetClient remoteClient;
  HttpResult retVal = HTTP_NO_ACTION;
  
  // listen for incoming clients
  remoteClient = ethServer.available();
  if (!remoteClient)
    return retVal;
    
  while (remoteClient.connected()) {
    if (remoteClient.available()) {
      char c = remoteClient.read();

      if (c != '\n' && c != '\r') {
        //read char by char HTTP request
        if (srvBufferPos < srvBufferSize) {

          //store characters to string 
          srvBuffer[srvBufferPos] = c;
          ++srvBufferPos;
        } 
        else
        {
          srvBufferPos = 0;
        }
        continue;
      }
    
      srvBuffer[srvBufferPos] = '\0';
      ///////////////
      //Serial.println(srvBuffer);

      //now output HTML data header
      // Parse get request
      char* ipStr, *portStr, *pch, *userStr, *pwdStr;
      byte counter;
      /////////////////////
      ipStr = strstr(srvBuffer, "ip=");
      portStr = strstr(srvBuffer, "port=");
      userStr = strstr(srvBuffer, "user=");
      pwdStr = strstr(srvBuffer, "pwd=");
	  
      // Call application specific parameters.
	  if(regHandler(srvBuffer, srvBufferPos))
	  {
		  retVal = retVal == HTTP_MQTT_CHANGE ? HTTP_ALL_CHANGE : HTTP_USER_CHANGE;
	  }
	  
      if(ipStr != NULL)
      {
        counter = 0;
        ipStr += 3;
        Serial.print(F("ip: "));
        pch = strtok (ipStr, ".&");
        while (pch != NULL && counter < 4)
        {
		  if(!isdigit(pch[0]))
		  {
			  break;
		  }
          Serial.print(pch);
          if(counter<3)Serial.print(F("."));
          else Serial.println(F(""));
          settings.mqtt_ip[counter] = atoi(pch);
          // go to next token
          pch = strtok (NULL, ".&");
          ++counter;
        }
		retVal = HTTP_MQTT_CHANGE;
      }
      if(portStr != NULL)
      {
        portStr += 5;
        Serial.print(F("port: "));
        pch = strtok (portStr, "&");
        Serial.println(pch);
        settings.mqtt_port = atoi(pch);
		retVal = HTTP_MQTT_CHANGE;
      }
      if(userStr != NULL)
      {
        userStr += 5;
        Serial.print(F("user: "));
        pch = strtok (userStr, "&");
        Serial.println(pch);
        strcpy(settings.mqtt_username, pch);
		retVal = HTTP_MQTT_CHANGE;
      }
      if(pwdStr != NULL)
      {
        pwdStr += 4;
        Serial.print(F("pwd: "));
        pch = strtok (pwdStr, "&");
        Serial.println(pch);
        strcpy(settings.mqtt_password, pch);
		retVal = HTTP_MQTT_CHANGE;
      }
	  
      // Respond with current configuration
      remoteClient.println(F("HTTP/1.1 200 OK"));
      remoteClient.println(F("Content-Type: text/html"));
      remoteClient.println(F(""));

      remoteClient.println(F("<HTML>"));
      remoteClient.println(F("<HEAD>"));
      remoteClient.println(F("<TITLE>Controller setup</TITLE>"));
      remoteClient.println(F("</HEAD>"));
      remoteClient.println(F("<BODY>"));

      remoteClient.println(F("<PRE>"));
      remoteClient.print(F("<H1>Settings:</H1>"));
      remoteClient.print(F("MQTT ip: \t"));
      for(uint8_t i = 0; i < 4; ++i)
      {
        remoteClient.print(settings.mqtt_ip[i]);
        if(i < 3)remoteClient.print(".");
        else remoteClient.println("");
      }
      remoteClient.print(F("MQTT port: \t"));
      remoteClient.println(settings.mqtt_port);
      remoteClient.print(F("MQTT user: \t"));
      remoteClient.println((const char*)settings.mqtt_username);
      remoteClient.print(F("MQTT pwd: \t"));
      remoteClient.println((const char*)settings.mqtt_password);
      
	  // Add any application specific data
	  respBuilder(remoteClient);
      
      remoteClient.println(F("</PRE>"));
      
      remoteClient.flush();
      //stopping client
      remoteClient.stop();
      //clearing string for next read
      srvBufferPos = 0;
    }
  }
  return retVal;
}