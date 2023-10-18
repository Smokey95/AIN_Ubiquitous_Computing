// Exercise 1 RGB

#include <WiFiNINA.h>

const int delayTime = 500; // Delay time in milliseconds (0.5 seconds)

void setup() {
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
}

void loop() {
  blink_red();
  delay(delayTime);
  blink_blue();
  delay(delayTime);
  blink_green();
  delay(delayTime);
}

void blink_red(){
  Serial.print("Red\n");
  digitalWrite(LEDR, HIGH);   // Red
  digitalWrite(LEDB, LOW);    // Blue
  digitalWrite(LEDG, LOW);    // Green
}

void blink_blue(){
  Serial.print("Blue\n");     
  digitalWrite(LEDR, LOW);    // Red
  digitalWrite(LEDB, HIGH);   // Blue
  digitalWrite(LEDG, LOW);    // Green
}

void blink_green(){
  Serial.print("Green\n");
  digitalWrite(LEDR, LOW);    // Red
  digitalWrite(LEDB, LOW);    // Blue
  digitalWrite(LEDG, HIGH);   // Green
}