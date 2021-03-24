
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "esp_log.h"
#include <string>
#include <lwip/netdb.h>


#define PORT 8090


static void udp_server_task(void *pvParameters)
{
    static const char *TAG = "UDP";

    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if(err < 0) ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);

        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        while (1) {
            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            } else {
                // Get the sender's ip address as string
                inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...

                std::string data(rx_buffer);

                while (!data.empty()) {
                    int separator = data.find(';');
                    std::string parse = data.substr(1, separator - 1); 

                    switch (data.front())
                    {
                    case 'L':
                        // left = std::atoi(parse.c_str());
                        // printf("Left: %d\n", left);
                        break;

                    case 'R':
                        // right = std::atoi(parse.c_str());
                        // printf("Right: %d\n", right);
                        break;

                    default:
                        printf("Unkown data: %s\n", parse.c_str());
                        break;
                    }
                    if(separator == -1) data.clear();
                    else data.erase(data.begin(), data.begin() + separator + 1);
                }

                // int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                // if (err < 0) {
                //     ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                //     break;
                // }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}