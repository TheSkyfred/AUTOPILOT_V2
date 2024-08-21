#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include "def.h"

#include "config.h"

#ifdef ESPNOW

#include <esp_now.h> //TELEMETRY
#include <WiFi.h>    //TELEMETRY



void init_telemetry();
void update_telemetry();



void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);

#endif

#endif /* TELEMETRY_H_ */