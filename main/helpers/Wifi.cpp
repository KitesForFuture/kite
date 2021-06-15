#include "./Wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_now.h"

array<uint8_t , 6> Wifi::destination {0};

void Wifi::init(array<uint8_t, 6> destination_mac) {

    destination = destination_mac;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_protocol(static_cast<wifi_interface_t>(ESP_IF_WIFI_STA), WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR) );
    ESP_ERROR_CHECK( esp_now_init() );

    esp_now_peer_info peer {
            .peer_addr {destination[0], destination[1], destination[2], destination[3], destination[4], destination[5]},
            .channel = 0,
            .ifidx = static_cast<wifi_interface_t>(ESP_IF_WIFI_STA),
            .encrypt = false,
    };
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

void Wifi::send(uint8_t *data, size_t len) {
    esp_err_t result = esp_now_send((const uint8_t*) &destination, data, len);

    if (result == ESP_OK) {}
    else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        printf("ESPNOW not Init.");
    }
    else if (result == ESP_ERR_ESPNOW_ARG) {
        printf("Invalid Argument");
    }
    else if (result == ESP_ERR_ESPNOW_INTERNAL) {
        printf("Internal Error");
    }
    else if (result == ESP_ERR_ESPNOW_NO_MEM) {
        printf("ESP_ERR_ESPNOW_NO_MEM");
    }
    else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
        printf("Peer not found.");
    }
    else {
        printf("Wifi send error.");
    }
}
