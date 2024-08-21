#include "telemetry.h"

#include "def.h"
#include "config.h"

#ifdef ESPNOW

bool isConnected = false;


// Fill the MAC adress of the reciever board
uint8_t RxMACaddress[] = {0x7D, 0x9E, 0xBD, 0xD9, 0xA0, 0xFD};
// LOLIN : 7C:9E:BD:D9:A0:FD

void init_wifi(){
    // Configurer l'ESP32 en point d'accès WiFi
  WiFi.softAP("MAVLink", "12345678");
 // udp.begin(14550);
}


void init_telemetry()
{
  WiFi.mode(WIFI_STA);
  //------------------------------------------------------------------------------------
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  //-------------------------------------------------------------------------------------
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(onDataRecv);
  //-------------------------------------------------------------------------------------
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, RxMACaddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  //-------------------------------------------------------------------------------------
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

  while (!isConnected)
  {
    Serial.println("Waiting for connection...");
    delay(1000);
  }

  Serial.println("Telemetry initialized successfully");
};





void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    Serial.print("Bytes received: ");
    Serial.println(len);
    isConnected = true;
}

void Telemetry_update()
{
  //-------------------------------------------------------------------------------------
  esp_err_t result = esp_now_send(RxMACaddress, (uint8_t *)&sentData, sizeof(sentData));
  //-------------------------------------------------------------------------------------
  if (result == ESP_OK)
    Serial.println("Sent with success");
  else
    Serial.println("Error sending the data");
  //-------------------------------------------------------------------------------------
}


// Callback function that will be executed when data is sent

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void update_telemetry(){}
#endif


