/**
 * @file    main.ino
 * @brief   Main file for the project
 * @author  Smokey95
*/

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <WiFiNINA.h>
#include "lib\credentials.h"

/*******************************************************************************
 * Constants
 ******************************************************************************/
const int PIN_RED_LED     = 2;        /**< GPIO pin for red LED */
const int PIN_BLUE_LED    = 3;        /**< GPIO pin for blue LED */  
const int PIN_GREEN_LED   = 4;        /**< GPIO pin for green LED */

/*******************************************************************************
 * Setup
 ******************************************************************************/
void setup() {
  // Initialize internal LED
  init_LED();

  // Initialize serial communication
  Serial.begin(9600);
}

/*******************************************************************************
 * Main
 ******************************************************************************/

void loop() {

}

/*******************************************************************************
 * Init Functions
 ******************************************************************************/

void init_LED() {
  // Initialize LEDs
  pinMode(PIN_RED_LED, OUTPUT);
  pinMode(PIN_BLUE_LED, OUTPUT);
  pinMode(PIN_GREEN_LED, OUTPUT);

  // Blink once to indicate start
  setColor(255, 0, 0);
  delay(250);
  setColor(0, 255, 0);
  delay(250);
  setColor(0, 0, 255);
  delay(250);
  setColor(0, 0, 0);
}

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

/**
 * @brief   Sets the color of the RGB LED
 * @param   R   Red value (0-255)
 * @param   G   Green value (0-255)
 * @param   B   Blue value (0-255)
*/
void setColor(int R, int G, int B) {
  analogWrite(PIN_RED_LED, 255 - R);
  analogWrite(PIN_GREEN_LED, 255 - G);
  analogWrite(PIN_BLUE_LED, 255 - B);
}