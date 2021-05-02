#pragma once

#include "esp_log.h"
#include <lwip/netdb.h>


#define KEEPALIVE_IDLE              1
#define KEEPALIVE_INTERVAL          2
#define KEEPALIVE_COUNT             2

extern QueueHandle_t batteryQueue;
extern QueueHandle_t distanceQueue;
extern QueueHandle_t speedQueue;
extern QueueHandle_t engineQueue;
extern QueueHandle_t accelQueue;
extern QueueHandle_t gyroQueue;


static void RXtcp(const int &sock);
static void TXtcp(void *);
static void TXudp(void *);
static void RXudp(void *);


static void RXtcp(const int &sock) {
    printf("TCP RX init\n");
    static const char *TAG = "TCP RX";
    int len;
    char rx_buffer[128];

    do {
        len = recv((int)sock, rx_buffer, sizeof(rx_buffer) - 1, 0);

        if (len < 0) ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        else if (!len) ESP_LOGW(TAG, "Connection closed");
        else {

            std::string data(rx_buffer, len);
            // printf("%s Received %d bytes: %s\n", TAG, len, data.c_str());


            // printf("TCP | Dostalem: %s\n", data.c_str());
            int separator = data.find(';');
            // printf("Znalazłem separator na miejscu: %d\n", separator);
            
            while(separator != std::string::npos) {
                std::string parse = data.substr(0, separator);
                data.erase(0, separator + 1);
                // printf("Nowy string: %s\n", parse.c_str());
                
                Packet* packet = Packet::decode(parse);
                if (packet != nullptr) {
                    switch (packet->getType())
                    {
                    case 'P': {
                        // printf("PING!\n");
                        std::string data = PingPacket().prepare();
                        send((int)sock, data.c_str(), data.size(), 0);
                        break;
                    }

                    default:
                        printf("Undefined UDP packet (%d bytes) --> %s \n", len, parse.c_str());
                        break;
                    }
                    delete packet;
                }
                separator = data.find(';');
                // printf("Znalazłem separator na miejscu: %d\n", separator);
            }
            // printf("Koniec wiadomości\n\n");
        }
    } while (len > 0);
    printf("TCP RX delete\n");
}


static void TXtcp(void* sock) { //sending
    int battery, distance;
    uint16_t speed;
    printf("TCP TX init | Socket: %d\n", (int)sock);

    while(true) {
        if(xQueueReceive(batteryQueue, &battery, 0) == pdTRUE) {
            std::string data = BatteryPacket(battery).prepare();
            send((int)sock, data.c_str(), data.size(), 0);
        }

        if(xQueueReceive(speedQueue, &speed, 0) == pdTRUE) {
            std::string data = SpeedPacket(speed, speed).prepare();
            send((int)sock, data.c_str(), data.size(), 0);
        }

        if(xQueueReceive(distanceQueue, &distance, 0) == pdTRUE) {
            std::string data = DistancePacket(distance).prepare();
            send((int)sock, data.c_str(), data.size(), 0);
        }
    }

    vTaskDelete(NULL);
}


static void TXudp(void *sock) {
    AcceloPacket accel;
    GyroPacket gyro;
    printf("UDP TX init | Socket: %d\n", (int)sock);

    while (true) {
        if(xQueueReceive(accelQueue, &accel, portMAX_DELAY) == pdTRUE) {
            printf("accel\n");
            std::string data = accel.prepare();
            send((int)sock, data.c_str(), data.size(), 0);
        }

        // if(xQueueReceive(gyroQueue, &gyro, 0) == pdTRUE) {
        //     std::string data = gyro.prepare();
        //     // send((int)sock, data.c_str(), data.size(), 0);
        // }
        // vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    vTaskDelete(NULL);
}



static void RXudp(void *port) {
    static const char *TAG = "UDP";

    char rx_buffer[128];
    // char clientAddress[128];
    struct sockaddr_in6 dest_addr;

    while (1) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons((int)port);

        int listen_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

        if (listen_sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }

        ESP_LOGI(TAG, "Socket created");

        int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if(err < 0) ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);

        ESP_LOGI(TAG, "Socket %d bound, port %d", listen_sock, (int)port);
        int len;
        // xTaskCreate(clientTX, "tcp_client_tx", 4096, (void*)listen_sock, 5, NULL);
        xTaskCreate(TXudp, "udp_server_tx", 4096, (void*)listen_sock, 5, NULL);

        while (1) {
            len = recv(listen_sock, rx_buffer, sizeof(rx_buffer) - 1, 0);

            if (len < 0) {
                ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
                break;
            } else {
                std::string data(rx_buffer, len);
                // printf("%s Received %d bytes |   %s\n", TAG, len, x.c_str());
    
                // printf("UDP | Dostalem: %s#\n", data.c_str());
                int separator = data.find(';');
                // printf("Znalazłem separator na miejscu: %d\n", separator);
                
                while(separator != std::string::npos) {
                    std::string parse = data.substr(0, separator);
                    data.erase(0, separator + 1);
                    // printf("Nowy string: %s\n", parse.c_str());

                    Packet* packet = Packet::decode(parse);

                    if (packet != nullptr) {
                        switch (packet->getType())
                        {
                        case 'E':
                            xQueueSendToBack(engineQueue, packet, 0);
                            break;
                        
                        default:
                            printf("Undefined UDP packet (%d bytes) --> %s \n", len, parse.c_str());
                            break;
                        }
                        delete packet;
                    }
                    separator = data.find(';');
                    // printf("Znalazłem separator na miejscu: %d\n", separator);
                }
            }
        }

        if(listen_sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(listen_sock, 0);
            close(listen_sock);
        }
    }

    vTaskDelete(NULL);
}