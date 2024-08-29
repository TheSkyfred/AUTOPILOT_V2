#ifndef MODES_H_
#define MODES_H_

#include "def.h"

// Modes principaux
enum FlightMode
{
  INIT,
  ARMED,
  TAKEOFF,
  CALIBRATE_SPEED,
  NAVIGATE,
  LANDING,
};

extern FlightMode currentMode;
const char *FlightModeToString(FlightMode currentMode);

// Modes pour le moteur

enum EngineStatus
{
  ENGINE_ON,
  ENGINE_OFF,
};

extern EngineStatus currentEngineStatus;


// Modes pour la vitesse
enum SpeedMode
{
  COEFFICIENT_CALC,
  SPEED_APPROACH,
  SPEED_PID_TUNING,
};

extern SpeedMode currentSpeedMode;

enum NavigationMode
{
  FLYING,
  STABILIZING,
  CORRECTING_HEADING,
  CORRECTING_ALTITUDE,
  RECOVERY,
};

extern NavigationMode currentNavigationMode;
const char *NavModeToString(NavigationMode currentNavigationMode);


// Modes pour les virages
enum TurnMode
{
  NO_TURN,
  BANKED_TURN,
  YAW_CORRECTION,
};

extern TurnMode currentTurnMode;
const char *TurnModeToString(TurnMode currentTurnMode);

enum LandingMode
{
INITIAL_APPROACH,
FINAL_APPROACH,
FLARE,
TOUCHDOWN,
};

extern LandingMode currentLandingMode;
#endif /* MODES_H_*/