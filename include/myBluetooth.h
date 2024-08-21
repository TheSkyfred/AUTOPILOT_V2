
#ifndef MYBLUETOOTH_H_
#define MYBLUETOOTH_H_

#include "def.h"

#include "config.h"

#ifdef MYBLUETOOTH
#include <BluetoothSerial.h>



void  init_BluetoothSerial();

void update_BluetoothSerial();

  #endif

#endif /* MYBLUETOOTH_H_ */