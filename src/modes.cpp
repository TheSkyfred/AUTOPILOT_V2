

#include "modes.h"

const char *FlightModeToString(FlightMode currentMode)
{
    switch (currentMode)
    {
    case INIT:
        return "INIT";
    case ARMED:
        return "ARMED";
    case TAKEOFF:
        return "TAKEOFF";
    case CALIBRATE_SPEED:
        return "CALIBRATE_SPEED";
    case NAVIGATE:
        return "NAVIGATE";
    case LANDING:
        return "LANDING";
    }
}

const char *NavModeToString(NavigationMode currentNavigationMode)
{
    switch (currentNavigationMode)
    {
    case FLYING:
        return "FLYING";
    case CORRECTING_HEADING:
        return "CORRECTING_HEADING";
    case CORRECTING_ALTITUDE:
        return "CORRECTING_ALTITUDE";
    case STABILIZING:
        return "STABILIZING";
    }
}

/*
  INIT,
  ARMED,
  TAKEOFF,
  CALIBRATE_SPEED,
  NAVIGATE,
  STABILIZING,
  CORRECTING_HEADING,
  CORRECTING_ALTITUDE,
  APPROACH,
  LANDING,

  */

/*

    case STABILIZING:
       return "STABILIZING";
   case CORRECTING_HEADING:
       return "CORRECTING_HEADING";
   case CORRECTING_ALTITUDE:
       return "CORRECTING_ALTITUDE";

   case APPROACH:
       return "APPROACH";
       */