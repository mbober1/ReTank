
#pragma once
#include "esp_log.h"
#include <lwip/netdb.h>
#include <packet.hpp>


extern QueueHandle_t engineQueue;
extern QueueHandle_t accelQueue;
extern QueueHandle_t gyroQueue;

static void udpServerTask(void *port) {
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

        ESP_LOGI(TAG, "Socket bound, port %d", (int)port);
        int len;
        // xTaskCreate(clientTX, "tcp_client_tx", 4096, (void*)listen_sock, 5, NULL);

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


void udpClientTX(const int &sock) {
    AcceloPacket accel;
    GyroPacket gyro;

    while (true) {
        if(xQueueReceive(accelQueue, &accel, 0) == pdTRUE) {
                std::string data = accel.prepare();
                send((int)sock, data.c_str(), data.size(), 0);
        }

        if(xQueueReceive(gyroQueue, &gyro, 0) == pdTRUE) {
                std::string data = gyro.prepare();
                send((int)sock, data.c_str(), data.size(), 0);
        }
    }
    
    vTaskDelete(NULL);
}