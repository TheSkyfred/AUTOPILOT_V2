

#include "barometer.h"

#include "def.h"
#include "config.h"

#include "mySerial.h"

//Barometer
Adafruit_BMP280 bmp;

float sea_pressure = 1012.00;

float barometer_temperature=0; // BMP280 TEMPERATURE IN CELCIUS
float current_barometer_altitude=0;  // BMP280 ALTITUDE IN METERS
bool barometer_status;
float altitude_error = 0;

void init_barometer()
{
  barometer_status = bmp.begin(barometer_address);
  if (!barometer_status)
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
    while (1)
      delay(10);
  }

  // Serial.println("BMP READY");
 // PrintoSerial("BMP SERIAL READY\n");
};



void update_barometer()
{
  barometer_temperature = bmp.readTemperature();    // reading temperature and sending to reciever
  current_barometer_altitude = bmp.readAltitude(sea_pressure); // adjust barometric pressure in your location

//  PrintoSerial("Température : %6.2f \n", barometer_temperature);
 // PrintoSerial("Altitude : %6.2f \n", current_barometer_altitude);
};

void check_altitude(){

altitude_error = abs(current_barometer_altitude - target_altitude);

};

