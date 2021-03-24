#include "nvs_flash.h"

#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include <cstring>
#include "secrets.hpp"
#include "esp_event.h"


static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    switch(event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;

        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
            break;

        case SYSTEM_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
            break;
    }
}




static void initialise_wifi(void)
{
    if (nvs_flash_init() == ESP_ERR_NVS_NO_FREE_PAGES) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    wifi_event_group = xEventGroupCreate();
    esp_netif_init();
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip);

    wifi_config_t wifi_config = {};
    
    strcpy((char*)wifi_config.sta.ssid, ssid);
    strcpy((char*)wifi_config.sta.password, password);
    
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();


    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(wifi_event_group);
}