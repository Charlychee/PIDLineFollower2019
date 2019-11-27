#include <WiFi.h>

#define SERVERPORT 55555

const char *SSID = "PIDDataServer";
const char *WiFiPassword = "P@sswordIs$3cure";

WiFiClient client;

void setup() {
  Serial.begin(115200);
  ConnectToWiFi();
  connectToServer();
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(client.available());
 if (client.available())
  {
    Serial.write((char)client.read());
  }
}

void ConnectToWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, WiFiPassword);
  //Serial.print("Connecting to "); Serial.println(SSID);
 
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    //Serial.print('.');
    delay(500);
 
    if ((++i % 16) == 0)
    {
      Serial.println(F(" still trying to connect"));
    }
  }
 
  Serial.print(F("Connected. My IP address is: "));
  Serial.println(WiFi.localIP());
}

void connectToServer()
{
  while (!client.connect(WiFi.gatewayIP(), SERVERPORT))
  {
  Serial.println("Error connecting to data server...");
    delay(500);
  }
    Serial.println("Connected to data server");
    client.flush();
    }
