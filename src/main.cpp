#include "tasks.hpp"



extern "C" void app_main()
{
    esp_pm_config_esp32_t power = {};
    power.min_freq_mhz = 240;
    power.max_freq_mhz = 240;
    power.light_sleep_enable = false;
    esp_pm_configure(&power);

    initialise_wifi();

    engineQueue = xQueueCreate(5, sizeof(EnginePacket));
    accelQueue = xQueueCreate(5, sizeof(AcceloPacket));
    gyroQueue = xQueueCreate(5, sizeof(GyroPacket));
    batteryQueue = xQueueCreate(5, sizeof(int));
    distanceQueue = xQueueCreate(5, sizeof(int));
    speedQueue = xQueueCreate(5, sizeof(int16_t));

    
    Ultrasonic sensor(TRIG, ECHO, SENSOR_PWM);



    static const char *TAG = "TCP";
    
    struct sockaddr_storage dest_addr;
    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons((int)TCP_PORT);

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
        ESP_LOGI(TAG, "Socket bound, port %d", (int)TCP_PORT);

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
            TaskHandle_t udp = NULL;
            TaskHandle_t driver = NULL;
            TaskHandle_t battery = NULL;
            TaskHandle_t mpu = NULL;

            xTaskCreate(clientTXtcp, "tcp_client_tx", 4096, (void*)sock, 5, &tx);
            xTaskCreate(udpServerTask, "udp_server", 4096, (void*)UDP_PORT, 10, &udp);
            xTaskCreate(robotDriver, "driver", 4096, nullptr, 20, &driver);
            xTaskCreate(batteryTask, "batteryTask", configMINIMAL_STACK_SIZE * 3, NULL, 3, &battery);
            xTaskCreate(mpuTask, "mpuTask", 4096, NULL, 5, &mpu);

            printf("dddd\r\n");
            clientRXtcp(sock); //wait until client connected

            printf("TCP TX kill task \n");
            vTaskDelete(&tx);
            vTaskDelete(&udp);
            vTaskDelete(&driver);
            vTaskDelete(&battery);
            vTaskDelete(&mpu);
            shutdown(sock, 0);
            close(sock);
        } 
    } else ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);

    close(listen_sock);
    // vTaskSuspend(NULL);
}