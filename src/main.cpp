#include <string>
#include <robot.hpp>
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include <cstring>

#include "secrets.hpp"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

// static EventGroupHandle_t wifi_event_group;
// #define WIFI_CONNECTED_BIT BIT0
// #define WIFI_FAIL_BIT      BIT1


// static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
// {
//     switch(event_id) {
//         case SYSTEM_EVENT_STA_START:
//             esp_wifi_connect();
//             break;

//         case SYSTEM_EVENT_STA_GOT_IP:
//             xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
//             break;

//         case SYSTEM_EVENT_STA_DISCONNECTED:
//             esp_wifi_connect();
//             xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
//             break;
//     }
// }

// static void initialise_wifi(void)
// {
//     if (nvs_flash_init() == ESP_ERR_NVS_NO_FREE_PAGES) {
//         nvs_flash_erase();
//         nvs_flash_init();
//     }

//     wifi_event_group = xEventGroupCreate();
//     esp_netif_init();
//     esp_log_level_set("wifi", ESP_LOG_WARN);
//     esp_event_loop_create_default();
//     esp_netif_create_default_wifi_sta();

//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     esp_wifi_init(&cfg);

//     esp_event_handler_instance_t instance_any_id;
//     esp_event_handler_instance_t instance_got_ip;
//     esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id);
//     esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip);

//     wifi_config_t wifi_config = {};
    
//     strcpy((char*)wifi_config.sta.ssid, ssid);
//     strcpy((char*)wifi_config.sta.password, password);
    
//     esp_wifi_set_mode(WIFI_MODE_STA);
//     esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
//     esp_wifi_start();


//     ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
//     ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
//     vEventGroupDelete(wifi_event_group);
// }


// static const char *TAG = "example";
// static const char *payload = "Message from ESP32 ";

// void udp_client_task2() {
//     char rx_buffer[128];
//     // char host_ip[] = HOST_IP_ADDR;
//     int addr_family = 0;
//     int ip_protocol = 0;
//     printf("czekam na WiFi\n");

//     while (!WIFI_CONNECTED_BIT)
//     {
//         printf(".");
//         vTaskDelay(200 / portTICK_PERIOD_MS);
//     }
    

//     while (1) {

//         struct sockaddr_in dest_addr;
//         dest_addr.sin_addr.s_addr = inet_addr("191.168.31.111");
//         dest_addr.sin_family = AF_INET;
//         dest_addr.sin_port = htons(8090);
//         addr_family = AF_INET;
//         ip_protocol = IPPROTO_IP;


//         int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
//         if (sock < 0) {
//             ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
//             break;
//         }

//         while (1) {
//             int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);

//             // Error occurred during receiving
//             if (len < 0) {
//                 ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
//                 break;
//             }
//             // Data received
//             else {
//                 rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
//                 ESP_LOGI(TAG, "Received %d bytes from host:", len);
//                 ESP_LOGI(TAG, "%s", rx_buffer);
//                 if (strncmp(rx_buffer, "OK: ", 4) == 0) {
//                     ESP_LOGI(TAG, "Received expected message, reconnecting");
//                     break;
//                 }
//             }

//             vTaskDelay(2000 / portTICK_PERIOD_MS);
//         }

//         if (sock != -1) {
//             ESP_LOGE(TAG, "Shutting down socket and restarting...");
//             shutdown(sock, 0);
//             close(sock);
//         }
//     }
//     vTaskDelete(NULL);
// }




// extern "C" void app_main()
// {	
//     initialise_wifi();
//     // xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
//     udp_client_task2();
// }




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

//   // }
// // }
int left;
int right;
int previousTime;
int previousTime2;
int16_t input[4];

extern "C" void app_main()
{
    robot Robot(IN1, IN2, PWM1, PWMCHANNEL, IN3, IN4, PWM2, ENC1A, ENC1B, ENC2A, ENC2B, PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2, PCNT_UNIT_3);

    while (1) {
        // Robot.autos();        

        vTaskDelay(pdMS_TO_TICKS(10));
        int64_t currentTime = esp_timer_get_time();
        if(currentTime - previousTime > 100000) {
            previousTime = currentTime;

            pcnt_get_counter_value(Robot.engine[0].encoder, &input[0]);
            pcnt_get_counter_value(Robot.engine[0].encoder2, &input[1]);
            pcnt_get_counter_value(Robot.engine[1].encoder, &input[2]);
            pcnt_get_counter_value(Robot.engine[1].encoder2, &input[3]);
            // printf("Setpoint: %d, ENC: %d, Error: %f\n", left, input1 + input2, e1);
            printf("L1: %d, L2: %d, R1: %d, R2: %d\n", input[0], input[1], input[2], input[3]);
        }

        if(currentTime - previousTime2 > 1000000) {
            previousTime2 = currentTime;
            left ++;
            right ++;
            if(left > 25) left = 0;
            if(right > 25) right = 0;
            Robot.setPoint(left, right);
        }
    }
}