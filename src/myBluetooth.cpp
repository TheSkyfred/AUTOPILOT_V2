#include "myBluetooth.h"


#ifdef MYBLUETOOTH


BluetoothSerial SerialBT;


void  init_BluetoothSerial() {
  SerialBT.begin("ESP32_MAVLink");  // Set the Bluetooth device name
  };


void update_BluetoothSerial();
#endif
