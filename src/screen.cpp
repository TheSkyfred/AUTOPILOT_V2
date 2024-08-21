#include "GPS.h"

#include "def.h"
#include "config.h"
#include "screen.h"

#include "modes.h"

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// Define the current and target values
int prev_current_roll, prev_current_pitch, prev_current_heading, prev_current_yaw;
int prev_target_roll, prev_target_pitch, prev_target_heading, prev_target_yaw;

float prev_GPS_altitude, prev_barometer_altitude;
float prev_target_altitude;

void init_screen()
{
  tft.begin();
  tft.setRotation(1);    // Set the screen rotation if needed
  tft.fillScreen(BLACK); // Set the background to black

  // Draw the circles
  drawCircle(CIRCLE1_X, CIRCLE1_Y, RADIUS, WHITE);
  drawCircle(CIRCLE2_X, CIRCLE2_Y, RADIUS, WHITE);
  drawCircle(CIRCLE3_X, CIRCLE3_Y, RADIUS, WHITE);
  drawCircle(CIRCLE4_X, CIRCLE4_Y, RADIUS, WHITE);

  /*
    // Draw the values
    drawValues(CIRCLE1_X, CIRCLE1_Y, RADIUS, current_roll, target_roll, RED, GREEN, true);
    drawValues(CIRCLE2_X, CIRCLE2_Y, RADIUS, current_pitch, target_pitch, RED, GREEN, true);
    drawValues(CIRCLE3_X, CIRCLE3_Y, RADIUS, current_heading, target_heading, RED, GREEN, false);
    drawValues(CIRCLE4_X, CIRCLE4_Y, RADIUS, current_yaw, target_yaw, RED, GREEN, true);
  */

  /*
    // Draw the titles with current values
    drawTitle(CIRCLE1_X, CIRCLE1_Y - RADIUS - 20, "ROLL", current_roll);
    drawTitle(CIRCLE2_X, CIRCLE2_Y - RADIUS - 20, "PITCH", current_pitch);
    drawTitle(CIRCLE3_X, CIRCLE3_Y - RADIUS - 20, "HEADING", current_heading);
    drawTitle(CIRCLE4_X, CIRCLE4_Y - RADIUS - 20, "YAW", current_yaw);
  */

  // Draw the gauges
  drawGauge(260, 40, 180, 500, "Altitude", RED);
  drawGaugeValues(260, 40, 180, 500, "GPS", GREEN);
  drawGaugeValues(290, 40, 180, 500, "BME", GREEN);

  // Draw correction bars
  drawCorrectionBar(CIRCLE1_X, CIRCLE1_Y + RADIUS + 10, 100, roll_correction, "ROLL");
  drawCorrectionBar(CIRCLE2_X, CIRCLE2_Y + RADIUS + 10, 100, pitch_correction, "PITCH");
  drawCorrectionBar(CIRCLE4_X, CIRCLE4_Y + RADIUS + 10, 100, yaw_correction, "YAW");

  // Initialize previous values
  prev_current_roll = current_roll;
  prev_target_roll = target_roll;
  prev_current_pitch = current_pitch;
  prev_target_pitch = target_pitch;
  prev_current_heading = current_heading;
  prev_target_heading = target_heading;
  prev_current_yaw = current_yaw;
  prev_target_yaw = target_yaw;
  prev_GPS_altitude = current_GPS_altitude;
  prev_barometer_altitude = current_barometer_altitude;
  prev_target_altitude = target_altitude;
}

void drawCircle(int x, int y, int radius, uint16_t color)
{
  tft.drawCircle(x, y, radius, color);
}

void drawValues(int x, int y, int radius, float currentValue, float targetValue, uint16_t currentColor, uint16_t targetColor, bool isDiameter)
{
  // Convert values to radians
  float currentRad = degreesToRadians(currentValue);
  float targetRad = degreesToRadians(targetValue);

  if (isDiameter)
  {
    // Calculate end points for the current value line (diameter)
    int currentX1 = x + radius * cos(currentRad);
    int currentY1 = y + radius * sin(currentRad);
    int currentX2 = x + radius * cos(currentRad + PI);
    int currentY2 = y + radius * sin(currentRad + PI);

    // Calculate end points for the target value line (diameter)
    int targetX1 = x + radius * cos(targetRad);
    int targetY1 = y + radius * sin(targetRad);
    int targetX2 = x + radius * cos(targetRad + PI);
    int targetY2 = y + radius * sin(targetRad + PI);

    // Draw the lines
    tft.drawLine(currentX1, currentY1, currentX2, currentY2, currentColor);
    tft.drawLine(targetX1, targetY1, targetX2, targetY2, targetColor);
  }
  else
  {
    // Calculate end point for the current value line (radius)
    int currentX = x + radius * cos(currentRad);
    int currentY = y + radius * sin(currentRad);

    // Calculate end point for the target value line (radius)
    int targetX = x + radius * cos(targetRad);
    int targetY = y + radius * sin(targetRad);

    // Draw the lines
    tft.drawLine(x, y, currentX, currentY, currentColor);
    tft.drawLine(x, y, targetX, targetY, targetColor);
  }
}

void drawGauge(int x, int y, int height, int GaugemaxValue, const char *title, uint16_t color)
{
  tft.drawRect(x, y, 20, height, WHITE); // Draw the gauge outline

  // Draw the title
  tft.setCursor(x - 30, y - 30);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.print(title);

  // Draw the graduations
  for (int i = 0; i <= GaugemaxValue; i += 50)
  {
    int yPos = y + height - map(i, 0, GaugemaxValue, 0, height);
    tft.drawLine(x, yPos, x - 5, yPos, WHITE);
    tft.setCursor(x - 30, yPos - 5);
    tft.setTextColor(WHITE);
    tft.setTextSize(1);
    tft.print(i);
  }
}

void drawGaugeValues(int x, int y, int height, int GaugemaxValue, const char *title, uint16_t color)
{
  tft.drawRect(x, y, 20, height, WHITE); // Draw the gauge outline

  // Draw the title
  tft.setCursor(x - 0, y - 20);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.print(title);
}

void drawTitle(int x, int y, const char *title, float current_value, float target_value)
{
  // Clear previous text area
  tft.fillRect(x - 60, y - 20, 120, 30, BLACK);

  // Draw title
  tft.setCursor(x - 55, y); // Adjusted for longer title + value
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.print(title);

  // Draw values
  tft.setCursor(x + 0, y); // Adjusted for longer title + value
  tft.setTextColor(RED);   // Assuming current values are shown in red
  tft.print((int)current_value);

  tft.setCursor(x + 20, y); // Adjusted for longer title + value
  tft.setTextColor(GREEN);  // Assuming current values are shown in red
  tft.print((int)target_value);
}

float degreesToRadians(float degree)
{
  return degree * PI / 180;
}

void updateGauge(int x, int y, int height, int GaugemaxValue, float currentValue, float targetValue, uint16_t currentColor, uint16_t targetColor)
{
  // Erase previous gauge values
  tft.fillRect(x + 1, y + 1, 18, height - 2, BLACK);

  // Draw current value
  int currentY = y + height - map(currentValue, 0, GaugemaxValue, 0, height);
  tft.fillRect(x + 1, currentY, 8, y + height - currentY, currentColor);

  // Draw target value
  int targetY = y + height - map(targetValue, 0, GaugemaxValue, 0, height);
  tft.fillRect(x + 11, targetY, 8, y + height - targetY, targetColor);

  tft.fillRect(x, y - 10, 100, 9, BLACK);
  tft.setCursor(x, y - 11); // Adjusted for longer title + value
  tft.setTextColor(RED);     // Assuming current values are shown in red
  tft.print((int)currentValue);
}

void update_screen()
{

  // Update corrections
  roll_correction = ((current_roll - SERVOMID) / (SERVOMAX - SERVOMIN)) * 100;
  pitch_correction = ((current_pitch - SERVOMID) / (SERVOMAX - SERVOMIN)) * 100;
  yaw_correction = ((current_yaw - SERVOMID) / (SERVOMAX - SERVOMIN)) * 100;

  // Erase previous lines
  drawValues(CIRCLE1_X, CIRCLE1_Y, RADIUS, prev_current_roll, prev_target_roll, BLACK, BLACK, true);
  drawValues(CIRCLE2_X, CIRCLE2_Y, RADIUS, prev_current_pitch, prev_target_pitch, BLACK, BLACK, true);
  drawValues(CIRCLE3_X, CIRCLE3_Y, RADIUS, prev_current_heading, prev_target_heading, BLACK, BLACK, false);
  drawValues(CIRCLE4_X, CIRCLE4_Y, RADIUS, prev_current_yaw, prev_target_yaw, BLACK, BLACK, true);

  // Draw the circles
  drawCircle(CIRCLE1_X, CIRCLE1_Y, RADIUS, WHITE);
  drawCircle(CIRCLE2_X, CIRCLE2_Y, RADIUS, WHITE);
  drawCircle(CIRCLE3_X, CIRCLE3_Y, RADIUS, WHITE);
  drawCircle(CIRCLE4_X, CIRCLE4_Y, RADIUS, WHITE);

  // Draw the values
  drawValues(CIRCLE1_X, CIRCLE1_Y, RADIUS, current_roll, target_roll, RED, GREEN, true);
  drawValues(CIRCLE2_X, CIRCLE2_Y, RADIUS, current_pitch, target_pitch, RED, GREEN, true);
  drawValues(CIRCLE3_X, CIRCLE3_Y, RADIUS, current_heading, target_heading, RED, GREEN, false);
  drawValues(CIRCLE4_X, CIRCLE4_Y, RADIUS, current_yaw, target_yaw, RED, GREEN, true);

  // Update titles
  drawTitle(CIRCLE1_X, CIRCLE1_Y - RADIUS - 10, "ROLL", current_roll, target_roll);
  drawTitle(CIRCLE2_X, CIRCLE2_Y - RADIUS - 10, "PITCH", current_pitch, target_pitch);
  drawTitle(CIRCLE3_X, CIRCLE3_Y - RADIUS - 10, "HEADING", current_heading, target_heading);
  drawTitle(CIRCLE4_X, CIRCLE4_Y - RADIUS - 10, "YAW", current_yaw, target_yaw);

  // Afficher la valeur actuelle de currentMode dans le coin inférieur gauche
  displayCurrentMode();

  // Update gauges
  updateGauge(260, 40, 180, 500, current_GPS_altitude, target_altitude, RED, GREEN);
  updateGauge(290, 40, 180, 500, current_barometer_altitude, target_altitude, RED, GREEN);

  // Update correction bars
  drawCorrectionBar(CIRCLE1_X, CIRCLE1_Y + RADIUS + 10, 100, roll_correction, "ROLL");
  drawCorrectionBar(CIRCLE2_X, CIRCLE2_Y + RADIUS + 10, 100, pitch_correction, "PITCH");
  drawCorrectionBar(CIRCLE4_X, CIRCLE4_Y + RADIUS + 10, 100, yaw_correction, "YAW");

  // Save previous values
  prev_current_roll = current_roll;
  prev_target_roll = target_roll;
  prev_current_pitch = current_pitch;
  prev_target_pitch = target_pitch;
  prev_current_heading = current_heading;
  prev_target_heading = target_heading;
  prev_current_yaw = current_yaw;
  prev_target_yaw = target_yaw;
  prev_GPS_altitude = current_GPS_altitude;
  prev_barometer_altitude = current_barometer_altitude;
  prev_target_altitude = target_altitude;
}

void drawCorrectionBar(int x, int y, int width, float correction, const char *title)
{
  // Draw the bar outline
  tft.drawRect(x - width / 2, y, width, 15, WHITE);

  /*
  // Draw the title
  tft.setCursor(x - width / 2 - 20, y + 5);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.print(title);
  */

  // Calculate the correction percentage
  int correctionWidth = map(correction, -100, 100, 0, width);

  // Draw the correction bar
  if (correctionWidth < 0)
  {
    tft.fillRect(x + correctionWidth, y + 1, -correctionWidth, 15, RED); // Negative correction
  }
  else
  {
    tft.fillRect(x, y + 1, correctionWidth, 15, GREEN); // Positive correction
  }
}

void displayCurrentMode()
{
  // Définir les dimensions de l'écran
  int screenWidth = tft.width();
  int screenHeight = tft.height();

  // Taille du texte
  int textSize = 1;
  tft.setTextSize(textSize);

  // Calculer la hauteur du texte (dépend de la police utilisée)
  int textHeight = 8 * textSize; // Exemple: hauteur de texte = 8 pixels * taille du texte

  // Effacer la zone avant de réécrire le texte
  tft.fillRect(0, screenHeight - textHeight, screenWidth, textHeight, BLACK);

  // Positionner le curseur dans le coin inférieur gauche
  tft.setCursor(0, screenHeight - textHeight);

  // Définir la couleur du texte
  tft.setTextColor(WHITE);

  // Afficher la valeur de currentMode
  tft.print("Mode: ");
  tft.print(FlightModeToString(currentMode));
}