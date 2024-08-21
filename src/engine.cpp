#include "arduino.h"

#include "engine.h"

#include "def.h"
#include "config.h"

#include "GPS.h"
#include "myDriver.h"

#include "modes.h"

#include "stabilization.h"

// Variables pour la vitesse
double target_speed = 0; // Vitesse de consigne en km/h
double initialPWM = 0;   // PWM initial calculé avec le coefficient

double motorSpeedCommand = 0;     // Commande PWM pour l'ESC
double lastMotorSpeedCommand = 0; // Dernière commande PWM
const int motorSpeedMIN = 1000;
const int motorSpeedMAX = 2000;

double engine_coefficient = 0; // Coefficient global utilisé pour l'approche

// Paramètres PID
double engine_Kp = 2.0; // Gain proportionnel
double engine_Ki = 5.0; // Gain intégral
double engine_Kd = 1.0; // Gain dérivé

// Paramètres de limitation
const double maxChangeRate = 10; // Maximum changement de PWM par mise à jour (valeur à ajuster)

// Variables pour le calcul des coefficients
const double speedStep = 0.1;    // Diminution de la vitesse par étape
const int stableDuration = 2000; // Durée de stabilisation en millisecondes : 2 secondes
unsigned long stabilizationStart = 0;
bool stable = false;
double coefficients[10]; // Tableau pour stocker les coefficients calculés
int coefficientIndex = 0;

// Création de l'objet PID
PID enginePID(&current_GPS_speed, &motorSpeedCommand, &target_speed, engine_Kp, engine_Ki, engine_Kd, DIRECT);

void init_engine()
{
  // Initialisation des broches
  pinMode(ESCPIN, OUTPUT);

  // Initialiser le PID
  enginePID.SetMode(AUTOMATIC);
  enginePID.SetOutputLimits(motorSpeedMIN, motorSpeedMAX);

  // Configurer le mode de calcul des coefficients
  stabilizationStart = millis();
}

void calculate_coefficient()
{
  if (millis() - stabilizationStart >= stableDuration)
  {
    // Calculer le coefficient
    engine_coefficient = motorSpeedCommand / current_GPS_speed;
    coefficients[coefficientIndex++] = engine_coefficient;
    Serial.print("Coefficient calculé : ");
    Serial.println(engine_coefficient);

    // Réduire la vitesse du moteur de 10% et stabiliser
    motorSpeedCommand = constrain(motorSpeedCommand * 0.9, 1000, 2000);
    set_engine(motorSpeedCommand);
    Serial.print("Nouvelle commande PWM : ");
    Serial.println(motorSpeedCommand);

    // Réinitialiser le temps de stabilisation
    stabilizationStart = millis();

    // Passer à l'étape suivante ou finir le calcul des coefficients
    if (coefficientIndex >= sizeof(coefficients) / sizeof(coefficients[0]))
    {
      currentSpeedMode = SPEED_APPROACH;

      Serial.println("Calcul des coefficients terminé. Passer au mode APPROCHE.");
    }
  }
}

void approach_speed()
{
  initialPWM = target_speed / engine_coefficient; // Utiliser votre coefficient moyen
  initialPWM = constrain(initialPWM, motorSpeedMIN, motorSpeedMAX);
  set_engine(initialPWM);
  Serial.print("Mode Approche | PWM initial: ");
  Serial.println(initialPWM);
  currentSpeedMode = SPEED_PID_TUNING; // Passer au mode PID après l'approche
}

void PID_correction()
{
  enginePID.Compute();
  double change = motorSpeedCommand - lastMotorSpeedCommand;
  if (abs(change) > maxChangeRate)
  {
    motorSpeedCommand = lastMotorSpeedCommand + (change > 0 ? maxChangeRate : -maxChangeRate);
  }
  set_engine(motorSpeedCommand);
  lastMotorSpeedCommand = motorSpeedCommand;
  Serial.print("Mode PID | Vitesse consigne (km/h): ");
  Serial.print(target_speed);
  Serial.print(" | Vitesse actuelle (km/h): ");
  Serial.print(current_GPS_speed);
  Serial.print(" | Commande PWM: ");
  Serial.println(motorSpeedCommand);
}

void stopMotor()
{
  // Fonction pour arrêter le moteur
  // Implémentez le code pour arrêter le moteur ici
  Serial.println("Moteur arrêté");
}

void engine_update(float target_speed)
{
}