/**
 * Test file for testing GPIO output to test a RGB LED (common anode)
*/

#include <WiFiNINA.h>

const int PIN_RED_LED     = 2;
const int PIN_BLUE_LED    = 3;
const int PIN_GREEN_LED   = 4;

void setup() {
  // Initialize LEDs
  pinMode(PIN_RED_LED, OUTPUT);
  pinMode(PIN_BLUE_LED, OUTPUT);
  pinMode(PIN_GREEN_LED, OUTPUT);

  Serial.begin(9600);
}

void loop() {

  // Color RED
  setColor(255, 0, 0);
  delay(2000);

  // Color GREEN
  setColor(0, 255, 0);
  delay(2000);

  // Color BLUE
  setColor(0, 0, 255);
  delay(2000);
}

void setColor(int R, int G, int B) {
  analogWrite(PIN_RED_LED, 255 - R);
  analogWrite(PIN_GREEN_LED, 255 - G);
  analogWrite(PIN_BLUE_LED, 255 - B);
}