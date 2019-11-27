//#include <ETH.h>
#include <WiFi.h>
//#include <WiFiAP.h>
//#include <WiFiClient.h>
//#include <WiFiGeneric.h>
//#include <WiFiMulti.h>
//#include <WiFiScan.h>
//#include <WiFiServer.h>
//#include <WiFiSTA.h>
//#include <WiFiType.h>
//#include <WiFiUdp.h>

const char* ssid = "PIDDataServer";
const char* password = "P@sswordIs$3cure";

const uint ServerPort = 55555;
WiFiServer Server(ServerPort);

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("IP of Server: ");
  Serial.println(IP);
  
  Server.begin();
}

void loop() {
  CheckForConnections();
  if (Serial2.available()) {      // If anything comes in Serial (USB),
    sendData();
  }
}

WiFiClient RemoteClient;

void CheckForConnections()
{
  if (Server.hasClient())
  {
    // If we are already connected to another computer,
    // then reject the new connection. Otherwise accept
    // the connection.
    if (RemoteClient.connected())
    {
      Serial.println("Connection rejected");
      Server.available().stop();
    }
    else
    {
      Serial.println("Connection accepted");
      RemoteClient = Server.available();
    }
  }
}

void sendData()
{
  if (RemoteClient.connected())
  {
    while (Serial2.available())
    {
      char c = Serial2.read();
      Serial.write(c);
      RemoteClient.write(c);   // read it and send it out Serial1 (pins 0 & 1)
    }
  }
}
