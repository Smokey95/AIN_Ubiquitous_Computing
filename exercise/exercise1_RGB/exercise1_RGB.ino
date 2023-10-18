// Exercise 1 RGB

#include <WiFiNINA.h>

const int delayTime = 500; // Delay time in milliseconds (0.5 seconds)

void setup() {
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
}

void loop() {
  Serial.print("Red\n");
  digitalWrite(LEDR, HIGH);   // Red
  digitalWrite(LEDB, LOW);    // Blue
  digitalWrite(LEDG, LOW);    // Green
  delay(delayTime);
  Serial.print("Blue\n");
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDB, HIGH);
  digitalWrite(LEDG, LOW);
  delay(delayTime);
  Serial.print("Green\n");
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDB, LOW);
  digitalWrite(LEDG, HIGH);
  delay(delayTime);
}