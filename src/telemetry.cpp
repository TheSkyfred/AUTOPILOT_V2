/*

Waypoint Normal (MAV_CMD_NAV_WAYPOINT):

param1: Temps de maintien (en secondes) à ce waypoint.
param2: Rayon d'acceptation en mètres pour considérer que le waypoint est atteint.
param3: Indique si l'avion doit passer par ce waypoint (0) ou à une distance spécifiée.
param4: Angle de lacet désiré au waypoint.

Décollage (MAV_CMD_NAV_TAKEOFF):

param1: Inclinaison minimale ou désirée.
param2 à param4: Généralement non utilisés pour le décollage.
x, y, z: Latitude, longitude et altitude de décollage.

Atterrissage (MAV_CMD_NAV_LAND):

param1: Altitude d'abandon en cas d'atterrissage interrompu.
param2: Mode d'atterrissage de précision.
param3 et param4: Généralement non utilisés.
x, y, z: Latitude, longitude et altitude cible pour l'atterrissage.

*/

#include "telemetry.h"

#include "def.h"
#include "config.h"

#include "modes.h"
#include "GPS.h"

#include <EEPROM.h> // Bibliothèque pour gérer l'EEPROM
#include "waypoints.h"

#ifdef ESPNOW

bool isConnected = false;

// Fill the MAC adress of the reciever board
uint8_t RxMACaddress[] = {0x7D, 0x9E, 0xBD, 0xD9, 0xA0, 0xFD};
// LOLIN : 7C:9E:BD:D9:A0:FD

////////// MAVLINK //////////

MAVLinkMessageStructure MavLinkMessageData;

// Configuration MAVLink
uint8_t system_id = MAV_TYPE_FIXED_WING;
uint8_t component_id = MAV_COMP_ID_AUTOPILOT1;
uint8_t autopilot = MAV_AUTOPILOT_GENERIC;
uint8_t system_status = MAV_STATE_STANDBY;               // MAV_STATE_ACTIVE;
uint16_t base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED; // MAV_MODE_AUTO_ARMED
uint32_t custom_mode = 0;

uint16_t mission_count = 0;

mavlink_mission_item_t mission_items[100];
mavlink_message_t msg;
mavlink_status_t status; // Declaration of status
uint8_t packetSize;
uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

// Derniers temps d'envoi pour chaque type de message
unsigned long lastGlobalPositionSend = 0;
unsigned long lastHeartbeatSend = 0;
unsigned long lastBatteryStatusSend = 0;
unsigned long lastMissionStatusSend = 0;
unsigned long lastAttitudeSend = 0;

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

  // Timeout pour la connexion ESP-NOW
  const unsigned long timeoutMs = 10 * 1000; // Timeout de 5 secondes
  unsigned long startTime = millis();

  while (!isConnected && (millis() - startTime < timeoutMs))
  {
    Serial.println("Waiting for connection...");
    delay(100); // Délai court pour éviter de bloquer totalement, ajustable
  }

  if (isConnected)
  {
    Serial.println("Telemetry initialized successfully");
  }
  else
  {
    Serial.println("Connection timeout: Telemetry initialization failed");
  }
};

void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  Serial.print("Bytes received: ");
  Serial.println(len);
  isConnected = true;

  for (int i = 0; i < packetSize; i++)
  {
    if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status))
    {
      handleMavlinkMessage(msg);
    }
  }
}

// USELESS FUNCTION
void Telemetry_update()
{
  // Mettre à jour les données MAVLink_Message avec les valeurs actuelles du capteur et du système
  updateFlightData(MavLinkMessageData);
  // Preparer le message MAVLINK à envoyer et l'envoyer
  PrepareMavlinkMessage(MavLinkMessageData);
}

// Callback function that will be executed when data is sent

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Gestion des messages MAVLink reçus
void handleMavlinkMessage(mavlink_message_t &msg)
{
  switch (msg.msgid)
  {
  case MAVLINK_MSG_ID_HEARTBEAT:
    // Serial.println("Received HEARTBEAT");
    break;

  case MAVLINK_MSG_ID_MISSION_COUNT:
    // Recevoir le nombre de waypoints
    mavlink_mission_count_t mission_count_msg;
    mavlink_msg_mission_count_decode(&msg, &mission_count_msg);
    mission_count = mission_count_msg.count;
    Serial.println("WayPoints Count : ");
    Serial.println(mission_count_msg.count);

    // Envoyer une requête pour le premier waypoint
    sendMissionRequest(0);
    break;

  case MAVLINK_MSG_ID_MISSION_ACK:
    Serial.println("Received MISSION_ACK");
    break;

  case MAVLINK_MSG_ID_MISSION_ITEM_INT:
    // Recevoir un waypoint
    mavlink_mission_item_int_t mission_item;
    mavlink_msg_mission_item_int_decode(&msg, &mission_item);

    // Stocker les informations du waypoint
    MLWaypoints[mission_item.seq] = {
        mission_item.param1,
        mission_item.param2,
        mission_item.param3,
        mission_item.param4,
        mission_item.x * 1e-7,
        mission_item.y * 1e-7,
        mission_item.z,
        mission_item.seq,
        mission_item.command,
        mission_item.target_system,
        mission_item.target_component,
        mission_item.frame,
        mission_item.current,
        mission_item.autocontinue,
        mission_item.mission_type};

    // Enregistrer le waypoint dans l'EEPROM
    saveWaypointToEEPROM(MLWaypoints[mission_item.seq]);

    // Afficher les informations sur le moniteur série
    Serial.println("Waypoint Info:");
    Serial.print("Number: ");
    Serial.println(mission_item.seq);
    Serial.print("Command: ");
    Serial.println(mission_item.command);
    Serial.print("Lat: ");
    Serial.println(mission_item.x * 1e-7, 7);
    Serial.print("Lon: ");
    Serial.println(mission_item.y * 1e-7, 7);
    Serial.print("Alt: ");
    Serial.println(mission_item.z);
    Serial.print("Param1: ");
    Serial.println(mission_item.param1);
    Serial.print("Param2: ");
    Serial.println(mission_item.param2);
    Serial.print("Param3: ");
    Serial.println(mission_item.param3);
    Serial.print("Param4: ");
    Serial.println(mission_item.param4);
    Serial.print("Frame: ");
    Serial.println(mission_item.frame);
    Serial.print("Current: ");
    Serial.println(mission_item.current);
    Serial.print("Autocontinue: ");
    Serial.println(mission_item.autocontinue);

    /*
                // Afficher le waypoint reçu sur l'écran TFT
                displayWaypoint(waypoints[mission_item.seq]);
    */

    // Demander le waypoint suivant ou envoyer une confirmation si tous les waypoints sont reçus
    if (mission_item.seq < mission_count - 1)
    {
      sendMissionRequest(mission_item.seq + 1);
    }
    else
    {
      sendMissionAck();
    }
    break;

  case MAV_CMD_COMPONENT_ARM_DISARM:

    if (currentMode == ARMED)
    {
      currentMode = DISARMED;
    }
    else
    {
      currentMode = ARMED;
    }

    break;

  case MAV_CMD_NAV_TAKEOFF:
    currentMode = TAKEOFF;
    break;

  case MAV_CMD_NAV_LAND:

    // GO TO THE APPROACH WAYPOINT

    break;

  case MAV_CMD_DO_SET_HOME:
    GPS_set_home();
    break;

  default:
    Serial.print("Received message with ID ");
    Serial.println(msg.msgid);
    break;
  }
}

void updateFlightData(MAVLinkMessageStructure &MavLinkMessageData)
{
  MavLinkMessageData.time_boot_ms = millis();
  MavLinkMessageData.lat = current_latitude;           // Latitude en degrés * 1E7
  MavLinkMessageData.lon = current_longitude;          // Longitude en degrés * 1E7
  MavLinkMessageData.alt = current_barometer_altitude; // Altitude en millimètres
  MavLinkMessageData.relative_alt = 0;
  MavLinkMessageData.vx = 0; // Vitesse dans l'axe X en cm/s
  MavLinkMessageData.vy = 0;
  MavLinkMessageData.vz = 0;
  MavLinkMessageData.hdg = current_heading; // Cap en centi-degrés

  // Attitude
  MavLinkMessageData.roll = current_radian_roll;   // Roulis en radians
  MavLinkMessageData.pitch = current_radian_pitch; // Tangage en radians
  MavLinkMessageData.yaw = current_radian_yaw;     // Lacet en radians
  MavLinkMessageData.rollspeed = 0.0;
  MavLinkMessageData.pitchspeed = 0.0;
  MavLinkMessageData.yawspeed = 0.0;

  // État de la batterie
  MavLinkMessageData.battery_id = 0;
  MavLinkMessageData.battery_function = MAV_BATTERY_FUNCTION_ALL;
  MavLinkMessageData.battery_type = MAV_BATTERY_TYPE_LIPO;
  MavLinkMessageData.battery_temperature = 0; // 30.0 degrés Celsius
  MavLinkMessageData.voltages[0] = 0;         // Exemple de tension de cellule
  MavLinkMessageData.voltages[1] = 0;
  MavLinkMessageData.voltages[2] = 0;
  MavLinkMessageData.current_battery = 0;  // 10A
  MavLinkMessageData.current_consumed = 0; // Consommé 2000 mAh
  MavLinkMessageData.energy_consumed = -1;
  MavLinkMessageData.battery_remaining = 0;

  // État du système
  MavLinkMessageData.autopilot_mode = MAV_AUTOPILOT_GENERIC;
  MavLinkMessageData.base_mode = MAV_MODE_AUTO_ARMED;
  MavLinkMessageData.custom_mode = 0;
  MavLinkMessageData.system_status = MAV_STATE_ACTIVE;

  // Mission
  MavLinkMessageData.current_mission_seq = currentWPIndex; // Waypoint actuel
  MavLinkMessageData.mission_result = 0;                   //;MAV_MISSION_ACCEPTED;

  // GPS Status
  MavLinkMessageData.gps_fix_type = 3;                         // 3D Fix
  MavLinkMessageData.satellites_visible = satellites_quantity; // 10 satellites visibles

  // Vitesse air
  MavLinkMessageData.airspeed = 0;    // Vitesse de l'air en m/s
  MavLinkMessageData.groundspeed = 0; // Vitesse au sol en m/s
};

// Fonction principale d'envoi à intervalles réguliers
void PrepareMavlinkMessage(MAVLinkMessageStructure &MavLinkMessageMavLinkMessageData)
{
  // mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

  unsigned long currentMillis = millis();

  // Envoyer le heartbeat
  if (currentMillis - lastHeartbeatSend >= heartbeatInterval)
  {
    mavlink_msg_heartbeat_pack(system_id, component_id, &msg, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC, MavLinkMessageData.base_mode, MavLinkMessageData.custom_mode, MavLinkMessageData.system_status);
    lastHeartbeatSend = currentMillis;
    sendMavlinkMessage(&msg);
  }

  // Envoyer l'Attiude
  if (currentMillis - lastAttitudeSend >= globalPositionInterval)
  {
    mavlink_msg_attitude_pack(system_id, component_id, &msg, MavLinkMessageData.time_boot_ms, MavLinkMessageData.roll, MavLinkMessageData.pitch, MavLinkMessageData.yaw, MavLinkMessageData.rollspeed, MavLinkMessageData.pitchspeed, MavLinkMessageData.yawspeed);
    lastAttitudeSend = currentMillis;
    sendMavlinkMessage(&msg);
  }

  // Envoyer la position globale
  if (currentMillis - lastGlobalPositionSend >= globalPositionInterval)
  {
    mavlink_msg_global_position_int_pack(system_id, component_id, &msg, MavLinkMessageData.time_boot_ms, MavLinkMessageData.lat, MavLinkMessageData.lon, MavLinkMessageData.alt, MavLinkMessageData.relative_alt, MavLinkMessageData.vx, MavLinkMessageData.vy, MavLinkMessageData.vz, MavLinkMessageData.hdg);
    lastGlobalPositionSend = currentMillis;
    sendMavlinkMessage(&msg);
  }

  // Envoyer le statut de la batterie
  if (currentMillis - lastBatteryStatusSend >= batteryStatusInterval)
  {
    mavlink_msg_battery_status_pack(system_id, component_id, &msg, MavLinkMessageData.battery_id, MavLinkMessageData.battery_function, MavLinkMessageData.battery_type, MavLinkMessageData.battery_temperature, MavLinkMessageData.voltages, MavLinkMessageData.current_battery, MavLinkMessageData.current_consumed, MavLinkMessageData.energy_consumed, MavLinkMessageData.battery_remaining, 0, 0, 0, 0, 0);
    lastBatteryStatusSend = currentMillis;
    sendMavlinkMessage(&msg);
  }

  // Envoyer le statut de la mission
  if (currentMillis - lastMissionStatusSend >= missionStatusInterval)
  {
    mavlink_msg_mission_current_pack(system_id, component_id, &msg, MavLinkMessageData.current_mission_seq, MavLinkMessageData.current_mission_total, MavLinkMessageData.mission_state, MavLinkMessageData.mission_mode, MavLinkMessageData.mission_id, MavLinkMessageData.fence_id, MavLinkMessageData.rally_points_id);
    lastMissionStatusSend = currentMillis;
    sendMavlinkMessage(&msg);
  }
}

void sendMavlinkMessage(mavlink_message_t *msg)
{
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  int message_length = mavlink_msg_to_send_buffer(buffer, msg);

  esp_err_t result = esp_now_send(RxMACaddress, buffer, message_length);
  if (result == ESP_OK)
  {
    Serial.println("Message MAVLink envoyé via ESP-NOW.");
  }
  else
  {
    Serial.println("Erreur lors de l'envoi du message MAVLink.");
  }
}

// Fonction pour envoyer un message de Heartbeat
void sendHeartbeat()
{
  mavlink_msg_heartbeat_pack(system_id, component_id, &msg, MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC, MavLinkMessageData.base_mode, MavLinkMessageData.custom_mode, MavLinkMessageData.system_status);

  sendMavlinkMessage(&msg);
}

//////// WAYPOINTS FUNCTIONS ////////

// Envoyer une requête de waypoint
void sendMissionRequest(uint16_t seq)
{
  // mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_request_pack(system_id, component_id, &msg, system_id, component_id, seq, MAV_MISSION_TYPE_MISSION);
  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

  // Envoyer le message sérialisé via ESP-NOW
  esp_err_t result = esp_now_send(RxMACaddress, buffer, len);
}

// Envoyer une confirmation de réception de tous les waypoints
void sendMissionAck()
{
  // mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_ack_pack(system_id, component_id, &msg, system_id, component_id, 0, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_MISSION);
  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

  // Envoyer le message sérialisé via ESP-NOW
  esp_err_t result = esp_now_send(RxMACaddress, buffer, len);
}

#endif
