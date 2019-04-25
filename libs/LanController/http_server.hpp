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
// size of buffer used to capture HTTP requests
#ifndef HTTP_REQ_BUF_SZ
#define HTTP_REQ_BUF_SZ   150
#endif
// size of buffer that stores the incoming string               // the web page file on the SD card
static char HTTP_req[REQ_BUF_SZ] = {0}; // buffered HTTP request stored as null terminated string

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
  }
  return result;
}

struct CustomHandlers
{
  bool (*customProcess)(const char*);
  void (*customForms)(EthernetClient&);
  void (*customSend)(EthernetClient&);
  MqttSettings* mqtt_ptr;
};

HttpResult httpHandle2(EthernetServer& ethServer, struct CustomHandlers& handlers)
{
  HttpResult result = HTTP_NO_ACTION;
  EthernetClient client = ethServer.available();  // try to get client

  if (client) {  // got client?
      boolean currentLineIsBlank = true;
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
                  if (strstr(HTTP_req, "ajax_inputs") != 0) {
                      // send rest of HTTP header
                      client.println(F("Content-Type: text/xml"));
                      client.println(F("Connection: close"));
                      client.println();

                      Serial.println(HTTP_req);
                      // print the received text to the Serial Monitor window
                      // if received with the incoming HTTP GET string
                      /*if (GetText(txt_buf, TXT_BUF_SZ)) {
                        Serial.println(F("\r\nReceived Text:"));
                        Serial.println(txt_buf);
                      }*/
                      if(parseIp(HTTP_req, handlers.mqtt_ptr->mqtt_ip))
                      {
                        Serial.print(F("Received: ip\t\t:"));
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
                        Serial.print(F("Received: port\t\t:"));
                        Serial.print(handlers.mqtt_ptr->mqtt_port);
                        Serial.println(F(""));
                        result |= HTTP_MQTT_CHANGE;
                      }
                      if(parseUser(HTTP_req, handlers.mqtt_ptr->mqtt_username))
                      {
                        Serial.print(F("Received: user\t\t:"));
                        Serial.print(handlers.mqtt_ptr->mqtt_username);
                        Serial.println(F(""));
                        result |= HTTP_MQTT_CHANGE;
                      }
                      if(parsePassword(HTTP_req, handlers.mqtt_ptr->mqtt_password))
                      {
                        Serial.print(F("Received: pwd\t\t:"));
                        Serial.print(handlers.mqtt_ptr->mqtt_password);
                        Serial.println(F(""));
                        result |= HTTP_MQTT_CHANGE;
                      }
                      if(handlers.customProcess(HTTP_req))
                      {
                        result |= HTTP_USER_CHANGE;
                      }
                  }
                  else
                  {  // web page request
                      // send rest of HTTP header
                      client.println(F("Content-Type: text/html"));
                      client.println(F("Connection: keep-alive"));
                      client.println(F(""));
                      // send web page

                      client.println(F("<!DOCTYPE html>"));
                      client.println(F("<html lang=\"en\">"));
                      client.println(F("<head>"));
                      client.println(F("<meta charset=\"utf-8\">"));
                      client.println(F("<title>RELIO Lan Controller configuration</title>"));
                      client.println(F("<script>"));
                      client.println(F("  strText = \"\";"));
                      client.println(F("  function SendText()"));
                      client.println(F("  {"));
                      client.println(F("   nocache = \"&nocache=\" + Math.floor(Math.random() * 1000000);"));
                      client.println(F("   var request = new XMLHttpRequest();"));
                      client.println(F("   strText = \"&ip=\" + document.getElementById(\"txt_form\").form_ip.value;"));
                      client.println(F("   strText += \"&port=\" + document.getElementById(\"txt_form\").form_port.value;"));
                      client.println(F("   strText += \"&user=\" + document.getElementById(\"txt_form\").form_user.value;"));
                      client.println(F("   strText += \"&pwd=\" + document.getElementById(\"txt_form\").form_pwd.value;"));
                      /*client.println(F("   strText += \"&timers=\";"));
                      for(uint8_t i = 0; i < 8; ++i)
                      {
                        client.print(F("   strText += document.getElementById(\"txt_form\").form_timer"));client.print(i);client.print(F(".value;"));
                        if(i < 7)client.print(F("   strText += \",\";"));
                      }*/
                      handlers.customSend(client);
                      
                      client.println(F(""));
                      client.println(F("   request.open(\"GET\", \"ajax_inputs\" + strText + nocache, true);"));
                      client.println(F("   request.send(null);"));
                      client.println(F("  }"));
                      client.println(F("</script>"));
                      client.println(F("</head>"));
                      client.println(F("<body onload=\"GetArduinoIO()\">"));
                      client.println(F("<form id=\"txt_form\" name=\"frmText\">"));
                      client.println(F("<PRE>"));
                      client.println(F("<H1>Settings:</H1>"));
                      client.print(F("\tMQTT ip:\t"));
                      client.print(F("<input type=\"text\" name=\"form_ip\" maxlength=15 size=15  value=\""));
                        for(uint8_t i = 0; i < 4; ++i)
                        {
                          client.print(handlers.mqtt_ptr->mqtt_ip[i]);
                          if(i < 3)client.print(".");
                        }
                      client.println(F("\">"));
                      client.print(F("\tMQTT port:\t"));
                      client.println(F("<input type=\"text\" name=\"form_port\" maxlength=4 size=4 value=\"")); client.println(handlers.mqtt_ptr->mqtt_port); client.println(F("\">"));
                      client.print(F("\tMQTT user:\t"));
                      client.println(F("<input type=\"text\" name=\"form_user\" maxlength=16 size=16 value=\"")); client.println((const char*)handlers.mqtt_ptr->mqtt_username); client.println(F("\">"));
                      client.print(F("\tMQTT pwd:\t"));
                      client.println(F("<input type=\"text\" name=\"form_pwd\" maxlength=16 size=16 value=\"")); client.println((const char*)handlers.mqtt_ptr->mqtt_password); client.println(F("\">"));
                      /*client.println(F("\n\tTimeout in seconds:"));
                        for(uint8_t i = 0; i < 8; ++i)
                        {
                          client.print(F("\t  Channel "));client.print(i+1);
                          client.print(F(" <input type=\"text\" name=\"form_timer"));client.print(i); client.print(F("\"  maxlength=3 size=3 value=\"")); 
                          client.print(boardSettings.timers[i]);
                          client.println(F("\">"));
                        }*/
                      handlers.customForms(client);

                        
                      client.println(F(""));
                      client.println(F("\t<input type=\"submit\" value=\"Save\" onclick=\"SendText()\" />"));
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
                      client.println(F("</PRE>"));
                      client.println(F("</form>"));
                      client.println(F("</body>"));
                      client.println(F(""));
                      client.println(F("</html>"));
                  }
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
