#include <ESP8266WiFi.h>
#include "ThingSpeak.h" // always include thingspeak header file after other header files and custom macros
#include <string.h>
#include <stdio.h>

String ssid = "Your SSID";   // your network SSID (name) 
String pass = "Your Password";   // your network password
int keyIndex = 0;            // your network key Index number (needed only for WEP)
WiFiClient  client;

unsigned long myChannelNumber = 000000;			// replace 0000000 with your channel number
const char * myWriteAPIKey = XYZ;   // replace XYZ with your channel write API Key

String stm32_input;
float temperature, humidity;
float field1, field2;
int getData1, getData2, getData3, getData4, field1Val, field2Val;
unsigned long prev, curr;

String myStatus = "";

void setup() {
  Serial.begin(115200);  // Initialize serial
    
  WiFi.mode(WIFI_STA); 
  ThingSpeak.begin(client);  // Initialize ThingSpeak

  if(WiFi.status() != WL_CONNECTED)
  {
    while(WiFi.status() != WL_CONNECTED)
    {
      WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      delay(5000); 

    }
  }

  pinMode(16, OUTPUT);
}

void loop() {

  digitalWrite(16, !(digitalRead(16)));

  curr = millis();

  if(Serial.available())
  {
    stm32_input = Serial.readStringUntil('\n');
      
      /*getData1 = stm32_input.indexOf('"');                  // 1st quote
      getData2 = stm32_input.indexOf('"', getData1 + 1); // 2nd quote
      getData3 = stm32_input.indexOf('"', getData2 + 1); // 3rd quote
      getData4 = stm32_input.indexOf('"', getData3 + 1); // 4th quote
      Serial.println(stm32_input);

      if(getData1 != -1 && getData1 != -1 && getData1 != -1 && getData1 != -1)
      {
        ssid = stm32_input.substring(getData1 + 1, getData2);
        pass = stm32_input.substring(getData3 + 1, getData4);
        if(WiFi.status() != WL_CONNECTED)
        {
          while(WiFi.status() != WL_CONNECTED)
          {
            WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
            delay(5000); 

          }
        }
          Serial.write("WIFI CONNECTED\r\n");
      }*/

    field1Val = stm32_input.indexOf("field1=");
    field2Val = stm32_input.indexOf("field2=");

    if (field1Val != -1 && field2Val != -1) 
    {

        // Extract field1 value
      int ampersandPos = stm32_input.indexOf('&', field1Val);
      String field1Str = stm32_input.substring(field1Val + 7, ampersandPos);

        // Extract field2 value
      String field2Str = stm32_input.substring(field2Val + 7);

      field1 = field1Str.toFloat();
      field2 = field2Str.toFloat();

      Serial.print("Field1: ");
      Serial.println(field1);
      Serial.print("Field2: ");
      Serial.println(field2);
    }

  }
  temperature = field1;
  humidity = field2;

  // set the fields with the values
  ThingSpeak.setField(1, temperature);
  ThingSpeak.setField(2, humidity);
  
  // set the status
  ThingSpeak.setStatus(myStatus);
  
  // write to the ThingSpeak channel
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if(x == 200){
    Serial.println("Channel update successful.");
  }
  else
  {
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }

  delay(20000);
  
}
