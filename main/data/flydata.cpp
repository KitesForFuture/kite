//
// Created by Leonard Koll on 16.05.21.
//

#include <esp_now.h>
#include "flydata.h"

void Flydata::send() {
    uint8_t mac[] = {48, 174, 164, 157, 56, 141};
    esp_err_t result = esp_now_send(mac, (uint8_t *)(this), sizeof(Flydata));
    if (result == ESP_OK) {
        printf("Success");
    }
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
        printf("Not sure what happened");
    }
}
