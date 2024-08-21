#ifndef CONFIG_H_
#define CONFIG_H_

// SONAR
#define MYBLUETOOTH false
#define ESPNOW true

#define MYSERIAL true
#define SCREEN true

#define BUZZERPIN 27
// SENSORS I2C ADRESS
#define barometer_address 0x76
#define gyroscope_address 0x29
#define servodriver_address 0x40
#define battery_address 0x40

#define MAX_DISTANCE 400 // distance maximale en cm pour le HC-SR04
// GPS
#define GPS_BAUDRATE 9600 // The default baudrate of NEO-6M is 9600
// SERIAL
#define PRINT_INTERVAL 1000 // Interval in milliseconds

#define TFT_CS 15
#define TFT_RST 2 // Vous pouvez aussi utiliser -1 si le pin de reset n'est pas connecté
#define TFT_DC 4
#define TFT_MOSI 23
#define TFT_SCLK 18

#define ESCPIN 9

#endif /* BAROMETER_H_ */