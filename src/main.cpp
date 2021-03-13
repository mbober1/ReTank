// #include <WiFi.h>
#include <robot.hpp>
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"


// const uint16_t port = 8090;
// const char * host = "192.168.31.111";

// #define shortClick 400
// #define longClick  8000000

// #define ENC_A 21
// #define ENC_B 27
// int button = 0;

  // WiFiUDP client;

  // int left, right;
  // char packetBuffer[255];



// void setup() {
  // WiFi.mode(WIFI_STA);
  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {};

  // Serial.println(WiFi.localIP());

  // client.begin(port);

// }

// void loop() {
  // int packetSize = client.parsePacket();

  // if(packetSize) {
  //   client.read(packetBuffer, 255);
  //   std::string data(packetBuffer);
  //   // Serial.println(packetBuffer);
  //   while (!data.empty())
  //   {
  //       int separator = data.find(';');
  //       std::string parse = data.substr(1, separator - 1); 

  //       switch (data.front())
  //       {
  //       case 'L':
  //           left = std::atoi(parse.c_str());
  //           // printf("Left: %d\n", left);
  //           break;

  //       case 'R':
  //           right = std::atoi(parse.c_str());
  //           // printf("Right: %d\n", right);
  //           break;

  //       default:
  //           // printf("Unkown data: %s\n", parse.c_str());
  //           break;
  //       }
  //       if(separator == -1) data.clear();
  //       else data.erase(data.begin(), data.begin() + separator + 1);
  //   }

  // Robot.drive(left, right);

  // }
// }
int left;
int right;
int previousTime;
int previousTime2;

extern "C" void app_main()
{
    robot Robot(GPIO_NUM_5, GPIO_NUM_18, 19, 0, GPIO_NUM_17, GPIO_NUM_16, 4, 33, 26, 32, 27, PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2, PCNT_UNIT_3);

    while (1) {
        Robot.autos();        

        vTaskDelay(pdMS_TO_TICKS(10));
        int64_t currentTime = esp_timer_get_time();
        if(currentTime - previousTime > 100000) {
            previousTime = currentTime;
            printf("Setpoint: %d, ENC: %d, Error: %f\n", left, input1 + input2, e1);
        }

        if(currentTime - previousTime2 > 1000000) {
            previousTime2 = currentTime;
            left ++;
            right ++;
            if(left > 30) left = 0;
            if(right > 30) right = 0;
            Robot.setPoint(left, right);
        }
    }
}