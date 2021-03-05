#include <WiFi.h>
#include <robot.hpp>
#include "secrets.hpp"


const uint16_t port = 8090;
const char * host = "192.168.31.111";

WiFiUDP client;
robot Robot(16, 17, 4, 0, 5, 18, 22, 2);

int left, right;
char packetBuffer[255];


void connect() {
  Serial.printf("Connecting to %s ", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.print("\nConnected to WiFi! My IP: ");
  Serial.println(WiFi.localIP());
  Serial.printf("\nConnecting to %s\n", host);
  client.begin(port);
}



void setup() {
  Serial.begin(115200);
  connect();
}



void loop() {
  int packetSize = client.parsePacket();

  if(packetSize) {
    client.read(packetBuffer, 255);
    std::string data(packetBuffer);
    Serial.println(packetBuffer);
    while (!data.empty())
    {
        int separator = data.find(';');
        std::string parse = data.substr(1, separator - 1); 

        switch (data.front())
        {
        case 'L':
            left = std::atoi(parse.c_str());
            printf("Left: %d\n", left);
            break;

        case 'R':
            right = std::atoi(parse.c_str());
            printf("Right: %d\n", right);
            break;

        default:
            printf("Unkown data: %s\n", parse.c_str());
            break;
        }
        if(separator == -1) data.clear();
        else data.erase(data.begin(), data.begin() + separator + 1);
    }

  Robot.drive(left, right);

  }

}