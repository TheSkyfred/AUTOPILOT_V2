#ifndef SCREEN_H_
#define SCREEN_H_


#include "def.h"

#ifdef SCREEN


#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>




// Define the center points for the circles
#define CIRCLE1_X 55
#define CIRCLE1_Y 50  // Increased by 10 pixels
#define CIRCLE2_X 160
#define CIRCLE2_Y 50  // Increased by 10 pixels
#define CIRCLE3_X 55
#define CIRCLE3_Y 175  // Increased by 10 pixels
#define CIRCLE4_X 160
#define CIRCLE4_Y 175  // Increased by 10 pixels
#define RADIUS 35

// Define colors
#define BLACK ILI9341_BLACK
#define WHITE ILI9341_WHITE
#define RED ILI9341_RED
#define GREEN ILI9341_GREEN




void init_screen();
void update_screen();

void drawCircle(int x, int y, int radius, uint16_t color);

void drawValues(int x, int y, int radius, float currentValue, float targetValue, uint16_t currentColor, uint16_t targetColor, bool isDiameter);

void drawGauge(int x, int y, int height, int GaugemaxValue, const char* title, uint16_t color);
void drawGaugeValues(int x, int y, int width, int height, const char *title, uint16_t color);
void updateGauge(int x, int y, int height, int GaugemaxValue, float currentValue, float targetValue, uint16_t currentColor, uint16_t targetColor) ;

void drawTitle(int x, int y, const char* title, float value) ;

void updateDisplay();
float degreesToRadians(float degree);
void drawCorrectionBar(int x, int y, int width, float correction, const char* title);

void displayCurrentMode();


#endif

#endif /* SCREEN_H_ */