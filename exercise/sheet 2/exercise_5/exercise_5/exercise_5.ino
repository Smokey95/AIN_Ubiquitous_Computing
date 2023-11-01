#include <WiFiNINA.h>

void setup() {
  // Initalize LEDs
  pinMode(LEDR, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    String msg = Serial.readString();

    if(msg.compareTo("1") == false){
      digitalWrite(LEDR, HIGH);       // Red LED on
      Serial.println(1);              // Writeback status for dashboard
    } else {
      digitalWrite(LEDR, LOW);        // Red LED off
      Serial.println(0);              // Writeback status for dashboard
    }
  }
}

void printSerialIn(String msg){
  Serial.print("Received: ");
  Serial.print(msg);
  Serial.print(" With length: ");
  Serial.println(msg.length());
}
