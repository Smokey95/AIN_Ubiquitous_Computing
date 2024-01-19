/**
 * Test file for testing Serial communication
*/

#include <WiFiNINA.h>

void setup() {
  Serial.begin(9600);
}

void loop() {
  
  String input;

  if (Serial.available() > 0) {
    
    input = Serial.readStringUntil('\n');
    if (input.length() > 0) {
      input.trim();
      Serial.println("Received: " + input);
    }
  }
  delay(100);
}