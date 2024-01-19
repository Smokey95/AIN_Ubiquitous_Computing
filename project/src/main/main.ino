/**
 * @file    main.ino
 * @brief   Main file for the project
 * @author  Smokey95
*/

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <WiFiNINA.h>


/*******************************************************************************
 * Constants
 ******************************************************************************/
const int lock_gpio_out  = 2;                          /**< GPIO pin for lock */

/*******************************************************************************
 * Setup
 ******************************************************************************/
void setup() {
  // Initialize internal LED to indicate when unlocked
  pinMode(LEDR, OUTPUT);
  // Initialize GPIO for lock
  pinMode(lock_gpio_out, OUTPUT);
  // Initialize serial communication
  Serial.begin(9600);
}

/*******************************************************************************
 * Main
 ******************************************************************************/

void loop() {

}