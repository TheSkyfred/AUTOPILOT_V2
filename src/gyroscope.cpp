
#include "gyroscope.h"

#include "def.h"
#include "config.h"

#include <SimpleKalmanFilter.h>

SimpleKalmanFilter kalmanFilter(3.0, 0.5, 0.05); // Mesure de variance, Processus de variance, Estimation d'erreur initiale
//2, 2, 0.01 //Other parameters for KALMAN FILTER


/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, gyroscope_address, &Wire);

int current_pitch = 0;
int current_roll = 0;
int current_yaw = 0;

float heading = 0;
int current_heading = 0;

void init_gyroscope()
{

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  delay(100);
}

void update_gyroscope()
{

  // Variables pour stocker les données de capteur
  sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;

  // Lire les événements de quaternions
  imu::Quaternion quat = bno.getQuat(); // Lire les quaternions directement

  // Calculer roll et pitch
  float quatReal = quat.w();
  float quatI = quat.x();
  float quatJ = quat.y();
  float quatK = quat.z();

  current_pitch = asin(2.0f * (quatReal * quatI + quatJ * quatK)) * 57.32;
  current_roll = atan2(2.0f * (quatReal * quatJ - quatK * quatI), 1.0f - 2.0f * (quatI * quatI + quatJ * quatJ)) * 57.32;
  current_yaw = atan2(2.0f * (quatReal * quatK + quatI * quatJ), 1.0f - 2.0f * (quatJ * quatJ + quatK * quatK)) * 57.32;

  // Afficher le cap magnétique en degrés
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);

  heading = atan2(magnetometerData.magnetic.y, magnetometerData.magnetic.x); // Le cap magnétique est souvent dans l'axe X
  if (heading < 0)
    heading += 2 * PI;
  if (heading > 2 * PI)
    heading -= 2 * PI;

  double kalmanHeading = kalmanFilter.updateEstimate(heading);

  current_heading = kalmanHeading * 180 / PI;
}
