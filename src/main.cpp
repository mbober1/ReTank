#include <WiFi.h>
#include <robot.hpp>
#include "secrets.hpp"

const uint16_t port = 8090;
const char * host = "192.168.31.111";

#define shortClick 400
#define longClick  8000000

#define ENC_A 21
#define ENC_B 27
int button = 0;

  WiFiUDP client;
  robot Robot(16, 17, 4, 0, 5, 18, 22, 2);

  int left, right;
  char packetBuffer[255];

void IRAM_ATTR sram() {
  int64_t start =  esp_timer_get_time();
  int64_t stop = start;

  while (!((GPIO.in >> ENC_A) & 1)) {
      // if((GPIO.in >> ENC_B) & 1) dir = 1;
      // else dir = 0;
  }


  stop = esp_timer_get_time();
  uint32_t stopwatch = stop - start;

  if(stopwatch > shortClick && stopwatch < longClick) button++;
}


void setup() {
  attachInterrupt(ENC_A, sram, FALLING);


  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {};

  Serial.println(WiFi.localIP());

  client.begin(port);

}

void loop() {
  int packetSize = client.parsePacket();

  if(packetSize) {
    client.read(packetBuffer, 255);
    std::string data(packetBuffer);
    // Serial.println(packetBuffer);
    while (!data.empty())
    {
        int separator = data.find(';');
        std::string parse = data.substr(1, separator - 1); 

        switch (data.front())
        {
        case 'L':
            left = std::atoi(parse.c_str());
            // printf("Left: %d\n", left);
            break;

        case 'R':
            right = std::atoi(parse.c_str());
            // printf("Right: %d\n", right);
            break;

        default:
            // printf("Unkown data: %s\n", parse.c_str());
            break;
        }
        if(separator == -1) data.clear();
        else data.erase(data.begin(), data.begin() + separator + 1);
    }

  Robot.drive(left, right);

  }
}