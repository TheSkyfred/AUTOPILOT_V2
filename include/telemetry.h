#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include "def.h"

#include "config.h"





#ifdef ESPNOW
#include <MAVLink.h>

#include <esp_now.h> //TELEMETRY
#include <WiFi.h>    //TELEMETRY
#include <WiFiAP.h>
#include <WiFiUdp.h>


void init_telemetry();
void update_telemetry();



void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);

void handleMavlinkMessage(mavlink_message_t &msg);

void sendHeartbeat() ;

void sendMissionRequest(uint16_t seq);
void sendMissionAck();

#endif

#endif /* TELEMETRY_H_ */