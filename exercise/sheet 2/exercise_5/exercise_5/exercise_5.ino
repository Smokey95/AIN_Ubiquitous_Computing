#include <WiFiNINA.h>

const int button_pin = 4;

void setup() {
  // Initalize LEDs
  pinMode(LEDR, OUTPUT);

  // initialize the pushbutton pin
  pinMode(button_pin, INPUT);

  Serial.begin(9600);
}

void loop() {

  static int debounce = 0;
  static String msg = "";

  // put main code here, to run repeatedly:

  if (Serial.available() > 0) {
    msg = Serial.readString();
  }

  if (digitalRead(button_pin) == HIGH || 
      msg.compareTo("1") == false){
    digitalWrite(LEDR, HIGH);       // Red LED on
    Serial.println(1);              // Writeback status for dashboard
  } else {
    digitalWrite(LEDR, LOW);        // Red LED off
    Serial.println(0);
  }
  
  delay(100);
}