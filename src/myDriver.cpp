
#include "myDriver.h"
#include "def.h"

#include "config.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int servo_channels[] = {0, 1, 2, 3}; // Canaux des servos sur le PCA9685
int motor_channel = 12;



void init_servo()
{
  // Initialiser le PCA9685
  pwm.begin(servodriver_address);
  pwm.setPWMFreq(60); // Fréquence recommandée pour les servos
}

void set_servos(int RollOutput, int PitchOuput, int YawOutput)
{
  pwm.setPWM(1, 0, constrain(RollOutput, SERVOMIN, SERVOMAX)); // Servo de l'aile droite

  pwm.setPWM(2, 0, constrain(-RollOutput, SERVOMIN, SERVOMAX)); // Servo de l'aile gauche

  pwm.setPWM(3, 0, constrain(PitchOuput, SERVOMIN, SERVOMAX)); // Servo de la profondeur

  pwm.setPWM(4, 0, constrain(YawOutput, SERVOMIN, SERVOMAX)); // Servo de la gouverne de direction
}

void set_engine(int motorSpeedCommand){
    pwm.setPWM(0, 0, constrain(motorSpeedCommand, speed_min, speed_max)); // Servo de l'aile droite

}