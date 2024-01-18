/**
 * Test file for testing GPIO output to test door sensor
*/

#include <WiFiNINA.h>

const int DOOR_SENSOR_PIN  = 4;

void setup() {
  // Initialize LEDs
  pinMode(LEDR, OUTPUT);

  // Initialize GPIO
  pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);

  Serial.begin(9600);
}

void loop() {

  static int door_state       = 0;
  static int door_state_prev  = 0;

  door_state = digitalRead(DOOR_SENSOR_PIN);

  if (door_state == HIGH && door_state_prev == LOW) {
    door_state_prev = door_state;
    digitalWrite(LEDR, HIGH);       // Red LED on
    Serial.println("DOOR OPEN");
  } else if (door_state == LOW && door_state_prev == HIGH) {
    door_state_prev = door_state;
    digitalWrite(LEDR, LOW);        // Red LED off
    Serial.println("DOOR CLOSED");
  }
  

  delay(100);
}