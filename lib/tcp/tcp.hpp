#pragma once

#include "esp_log.h"
#include <lwip/netdb.h>
#include <packet.hpp>


#define KEEPALIVE_IDLE              1
#define KEEPALIVE_INTERVAL          2
#define KEEPALIVE_COUNT             2

extern QueueHandle_t batteryQueue;
extern QueueHandle_t distanceQueue;

void clientRX(const int &sock);
void clientTX(void* sock);

static void tcpServerTask(void* port);


void tcpServerTask(void* port) {
    static const char *TAG = "TCP";
    
    struct sockaddr_storage dest_addr;
    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons((int)port);

    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if(err) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", AF_INET);
    } else {
        err = listen(listen_sock, 1);
    }

    if(!err) {
        ESP_LOGI(TAG, "Socket bound, port %d", (int)port);

        while (1) {
            ESP_LOGI(TAG, "Socket listening");

            struct sockaddr_storage source_addr;
            socklen_t addr_len = sizeof(source_addr);
            int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
            if (sock < 0) {
                ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
                break;
            }

            // Set tcp keepalive option
            int keepAlive = 1;
            int keepIdle = KEEPALIVE_IDLE;
            int keepInterval = KEEPALIVE_INTERVAL;
            int keepCount = KEEPALIVE_COUNT;
            setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
            setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
            setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
            setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));


            // Convert ip address to string
            char clientAddress[128];
            if (source_addr.ss_family == PF_INET) {
                inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, clientAddress, sizeof(clientAddress) - 1);
            }

            printf("%s | Socket %d accepted ip address: %s\n", TAG, sock, clientAddress);

            TaskHandle_t tx = NULL;

            // xTaskCreate(clientTX, "tcp_client_tx", 4096, (void*)sock, 5, &tx);
            printf("dddd\r\n");
            clientRX(sock); //wait until client connected

            printf("TCP TX kill task \n");
            vTaskDelete(&tx);
            shutdown(sock, 0);
            close(sock);
        } 
    } else ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);

    close(listen_sock);
    vTaskDelete(NULL);
}

void clientRX(const int &sock) {
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
            printf("%s Received %d bytes: %s\n", TAG, len, data.c_str());


            printf("TCP | Dostalem: %s\n", data.c_str());
            int separator = data.find(';');
            printf("Znalazłem separator na miejscu: %d\n", separator);
            
            while(separator != std::string::npos) {
                std::string parse = data.substr(0, separator);
                data.erase(0, separator + 1);
                printf("Nowy string: %s\n", parse.c_str());
                
                Packet* packet = Packet::decode(parse);
                if (packet != nullptr) {
                    switch (packet->getType())
                    {
                    case 'P': {
                        printf("PING!\n");
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
                printf("Znalazłem separator na miejscu: %d\n", separator);
            }
            printf("Koniec wiadomości\n\n");
        }
    } while (len > 0);
    printf("TCP RX delete\n");
}


void clientTX(void* sock) { //sending
        int battery, distance;
        printf("TCP TX init | Socket: %d\n", (int)sock);

        while(true) {
            if(xQueueReceive(batteryQueue, &battery, 0) == pdTRUE) {
                std::string data = BatteryPacket(battery).prepare();
                send((int)sock, data.c_str(), data.size(), 0);
            }

            if(xQueueReceive(distanceQueue, &distance, 0) == pdTRUE) {
                // printf("Distance: %d\n", distance);
                std::string data = DistancePacket(distance).prepare();
                send((int)sock, data.c_str(), data.size(), 0);
            }
        }

    vTaskDelete(NULL);
}