/**
 * @file serial_gpio.ino
 * @brief Test file for testing Serial communication with GPIO control
*/
#include <WiFiNINA.h>

const int gpio_out = 2;                          /**< GPIO pin for lock */

void setup() {
  // Initialize LEDs
  pinMode(LEDR, OUTPUT);

  // Initialize GPIO
  pinMode(gpio_out, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  
  static int lock_state = 0;
  
  String input_msg;

  if (Serial.available() > 0) {
    input_msg = Serial.readStringUntil('\n');
    if (input_msg.length() > 0) {
      input_msg.trim();
      if (input_msg.compareTo("LOCK_UNLOCK") == 0) {
        lock_state = 1;
        Serial.println("STAT_LOCK_UNLOCK");
      } else if (input_msg.compareTo("LOCK_LOCK") == 0) {
        lock_state = 0;
        Serial.println("STAT_LOCK_LOCK");
      }
    }
  }

  if (lock_state){
    digitalWrite(LEDR, HIGH);       // Red LED on
    digitalWrite(gpio_out, HIGH);
  } else {
    digitalWrite(LEDR, LOW);        // Red LED off
    digitalWrite(gpio_out, LOW);
  }

  delay(1);
}

