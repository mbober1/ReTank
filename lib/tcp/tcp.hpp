#pragma once

#include "esp_log.h"
#include <lwip/netdb.h>
#include <packet.hpp>


#define KEEPALIVE_IDLE              1
#define KEEPALIVE_INTERVAL          2
#define KEEPALIVE_COUNT             3


static void clientConnected(const int sock);
static void tcpServerTask( void * pvParameters );


void tcpServerTask( void * pvParameters ) {
    static const char *TAG = "TCP";
    
    char addr_str[128];
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(8091);

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
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", AF_INET);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", 8091);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {
        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }

        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        clientConnected(sock);

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}




void clientConnected(const int sock)
{
    static const char *TAG = "TCP";
    int len;
    char rx_buffer[128];

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
    } while (len > 0);
}