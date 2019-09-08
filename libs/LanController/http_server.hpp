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
        //Serial.print(F("ip: "));
        pch = strtok (ipStr, ".&");
        while (pch != NULL && counter < 4)
        {
		  if(!isdigit(pch[0]))
		  {
			  break;
		  }
          //Serial.print(pch);
          //if(counter<3)Serial.print(F("."));
          //else Serial.println(F(""));
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
        //Serial.print(F("port: "));
        pch = strtok (portStr, "&");
        //Serial.println(pch);
        settings.mqtt_port = atoi(pch);
		retVal = HTTP_MQTT_CHANGE;
      }
      if(userStr != NULL)
      {
        userStr += 5;
        //Serial.print(F("user: "));
        pch = strtok (userStr, "&");
        //Serial.println(pch);
        strcpy(settings.mqtt_username, pch);
		retVal = HTTP_MQTT_CHANGE;
      }
      if(pwdStr != NULL)
      {
        pwdStr += 4;
        //Serial.print(F("pwd: "));
        pch = strtok (pwdStr, "&");
        //Serial.println(pch);
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
// size of buffer used to capture HTTP requests
#ifndef HTTP_REQ_BUF_SZ
#define HTTP_REQ_BUF_SZ   150
#endif

bool parseIp(const char* reqStr, uint8_t* ipArray)
{
  bool result = false;
  char* strPtr = strstr(reqStr, "&ip=");
  if(strPtr)
  {
    strPtr += 3;
    int  iArray[4];
    if(sscanf(strPtr, "=%d.%d.%d.%d&", iArray, iArray+1, iArray+2, iArray+3) == 4)
    {
      for(int i = 0; i < 4; ++i)
      {
        ipArray[i] = iArray[i];
      }
      result = true;
    }
  }
  return result;
}
bool parsePort(const char* reqStr, uint16_t* port)
{
  bool result = false;
  char* strPtr = strstr(reqStr, "&port=");
  if(strPtr)
  {
    strPtr += 5;
    int  i1;
    if(sscanf(strPtr, "=%d&", &i1) == 1)
    {
      *port = i1;
      result = true;
    }
  }
  return result;
}
bool parseUser(const char* reqStr, char* userStr)
{
  bool result = false;
  char* strPtr = strstr(reqStr, "&user=");
  if(strPtr)
  {
    strPtr += 6;
    while(*strPtr && *strPtr != '&')
    {
      *userStr = *strPtr;
      ++userStr;
      ++strPtr;
      result = true;
    }
	*userStr = 0;
  }
  return result;
}
bool parsePassword(const char* reqStr, char* pwdStr)
{
  bool result = false;
  char* strPtr = strstr(reqStr, "&pwd=");
  if(strPtr)
  {
    strPtr += 5;
    while(*strPtr && *strPtr != '&')
    {
      *pwdStr = *strPtr;
      ++pwdStr;
      ++strPtr;
      result = true;
    }
	*pwdStr = 0;
  }
  return result;
}

struct CustomHandlers
{
  bool (*customProcess)(const char*);
  void (*customForms)(EthernetClient&);
  MqttSettings* mqtt_ptr;
};

HttpResult httpHandle2(EthernetServer& ethServer, struct CustomHandlers& handlers, const IPAddress& localIp)
{
  // size of buffer that stores the incoming string               // the web page file on the SD card
  static char    HTTP_req[HTTP_REQ_BUF_SZ] = {0}; // buffered HTTP request stored as null terminated string
  int16_t req_index = 0;
  
  HttpResult result = HTTP_NO_ACTION;
  EthernetClient client = ethServer.available();  // try to get client

  if (client) {  // got client?
      bool currentLineIsBlank = true;
      while (client.connected()) {
          if (client.available()) {   // client data available to read
              char c = client.read(); // read 1 byte (character) from client
              // limit the size of the stored received HTTP request
              // buffer first part of HTTP request in HTTP_req array (string)
              // leave last element in array as 0 to null terminate string (HTTP_REQ_BUF_SZ - 1)
              if (req_index < (HTTP_REQ_BUF_SZ - 1)) {
                  HTTP_req[req_index] = c;          // save HTTP request character
                  req_index++;
              }
              // last line of client request is blank and ends with \n
              // respond to client only after last line received
              if (c == '\n' && currentLineIsBlank) {
                  HTTP_req[req_index] = 0;
                  
                  // send a standard http response header
                  client.println(F("HTTP/1.1 200 OK"));
                  // remainder of header follows below, depending on if
                  // web page or XML page is requested
                  // Ajax request - send XML file
                  if (strstr(HTTP_req, "GET /?") != 0) {
                      // send rest of HTTP header
                      //client.println(F("Content-Type: text/xml"));
                      //client.println(F("Connection: close"));
                      //client.println();
                      Serial.println(HTTP_req);

                      if(parseIp(HTTP_req, handlers.mqtt_ptr->mqtt_ip))
                      {
                        Serial.print(F("R: ip\t:"));
                        for(int i = 0; i < 4; ++i)
                        {
                          Serial.print(handlers.mqtt_ptr->mqtt_ip[i]);
                          if(i<3)Serial.print(F("."));
                        }
                        Serial.println(F(""));
                        result |= HTTP_MQTT_CHANGE;
                      }
                      if(parsePort(HTTP_req, &handlers.mqtt_ptr->mqtt_port))
                      {
                        Serial.print(F("R: port\t:"));
                        Serial.println(handlers.mqtt_ptr->mqtt_port);
                        Serial.println(F(""));
                        result |= HTTP_MQTT_CHANGE;
                      }
                      if(parseUser(HTTP_req, handlers.mqtt_ptr->mqtt_username))
                      {
                        Serial.print(F("R: user\t:"));
                        Serial.println(handlers.mqtt_ptr->mqtt_username);
                        Serial.println(F(""));
                        result |= HTTP_MQTT_CHANGE;
                      }
                      if(parsePassword(HTTP_req, handlers.mqtt_ptr->mqtt_password))
                      {
                        Serial.print(F("R: pwd\t:"));
                        Serial.println(handlers.mqtt_ptr->mqtt_password);
                        Serial.println(F(""));
                        result |= HTTP_MQTT_CHANGE;
                      }
                      if(handlers.customProcess(HTTP_req))
                      {
                        result |= HTTP_USER_CHANGE;
                      }
                  }
				  // web page request
				  // send rest of HTTP header
				  client.println(F("Content-Type: text/html"));
				  client.println(F("Connection: keep-alive"));
				  client.println(F(""));
				  // send web page

				  client.println(F("<!DOCTYPE html>"));
				  client.println(F("<html lang=\"en\">"));
				  client.println(F("<head>"));
				  client.println(F("<meta charset=\"utf-8\">"));
				  client.println(F("<title>Lan Controller configuration</title>"));
				  client.println(F("</head>"));
				  client.println(F("<body>"));
				  client.print(F("<form action=\"http://"));
				  client.print(localIp);
				  client.println(F("\" method=\"get\">"));
				  client.println(F("<PRE>"));
				  client.println(F("<H1>Settings:</H1>"));
				  client.print(F("  MQTT ip:\t"));
				  client.print(F("<input type=\"text\" name=\"ip\" maxlength=15 size=15  value=\""));
					for(uint8_t i = 0; i < 4; ++i)
					{
					  client.print(handlers.mqtt_ptr->mqtt_ip[i]);
					  if(i < 3)client.print(".");
					}
				  client.println(F("\">"));
				  client.print(F("  MQTT port:\t"));
				  client.println(F("<input type=\"text\" name=\"port\" maxlength=4 size=4 value=\"")); client.println(handlers.mqtt_ptr->mqtt_port); client.println(F("\">"));
				  client.print(F("  MQTT user:\t"));
				  client.println(F("<input type=\"text\" name=\"user\" maxlength=16 size=16 value=\"")); client.println((const char*)handlers.mqtt_ptr->mqtt_username); client.println(F("\">"));
				  client.print(F("  MQTT pwd:\t"));
				  client.println(F("<input type=\"text\" name=\"pwd\" maxlength=16 size=16 value=\"")); client.println((const char*)handlers.mqtt_ptr->mqtt_password); client.println(F("\">"));

				  handlers.customForms(client);
					
				  client.println(F(""));
				  client.println(F("\t<input type=\"submit\" value=\"Send\"/>"));
				  client.println(F(""));
				  
				  client.println(F("</PRE>"));
				  client.println(F("</form>"));
				  client.println(F("</body>"));
				  client.println(F("</html>"));

                  // reset buffer index and all buffer elements to 0
                  req_index = 0;
                  //StrClear(HTTP_req, REQ_BUF_SZ);
                  break;
              }
              // every line of text received from the client ends with \r\n
              if (c == '\n') {
                  // last character on line of received text
                  // starting new line with next character read
                  currentLineIsBlank = true;
              } 
              else if (c != '\r') {
                  // a text character was received from client
                  currentLineIsBlank = false;
              }
          } // end if (client.available())
      } // end while (client.connected())
      client.flush();
      delay(1);      // give the web browser time to receive the data
      client.stop(); // close the connection
  } // end if (client)
  return result;
}
