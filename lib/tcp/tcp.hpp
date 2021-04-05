#pragma once

#include "esp_log.h"
#include <lwip/netdb.h>
#include <packet.hpp>


#define KEEPALIVE_IDLE              1
#define KEEPALIVE_INTERVAL          2
#define KEEPALIVE_COUNT             2

extern QueueHandle_t batteryQueue;
extern QueueHandle_t distanceQueue;

static void clientConnected(const int sock);
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

            ESP_LOGI(TAG, "Socket accepted ip address: %s", clientAddress);

            clientConnected(sock);

            shutdown(sock, 0);
            close(sock);
        } 
    } else ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);

    close(listen_sock);
    vTaskDelete(NULL);
}




void clientConnected(const int sock)
{
    static const char *TAG = "TCP";
    int len;
    char rx_buffer[128];

    int battery, distance;

    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);

        if (len < 0) ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        else if (!len) ESP_LOGW(TAG, "Connection closed");
        else {

            std::string x(rx_buffer, len);
            printf("%s Received %d bytes: %s\n", TAG, len, x.c_str());

            Packet* packet = Packet::decode(x);

            if (packet != nullptr) {
                switch (packet->getType())
                {
                case 'P': {
                    printf("PING!\n");
                    std::string data = PingPacket().prepare();
                    send(sock, data.c_str(), data.size(), 0);
                    break;
                }

                case 'E':
                    printf("ENGINE L: %d | P: %d\n", ((EnginePacket*)packet)->left, ((EnginePacket*)packet)->right);
                    break;
                
                default:
                    printf("du[pa!\n");
                    break;
                }
                delete packet;
            }
        }

        //sending
        if(xQueueReceive(batteryQueue, &battery, 0) == pdTRUE) {
            std::string data = BatteryPacket(battery).prepare();
            send(sock, data.c_str(), data.size(), 0);
        }

        if(xQueueReceive(distanceQueue, &distance, 0) == pdTRUE) {
            std::string data = DistancePacket(distance).prepare();
            send(sock, data.c_str(), data.size(), 0);
        }



    } while (len > 0);
}