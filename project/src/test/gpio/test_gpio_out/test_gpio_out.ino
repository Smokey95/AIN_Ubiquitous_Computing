/**
 * Test file for testing GPIO output to test electrical relay to control lock
*/

#include <WiFiNINA.h>

const int gpio_out  = 2;

void setup() {
  // Initialize LEDs
  pinMode(LEDR, OUTPUT);

  // Initialize GPIO
  pinMode(gpio_out, OUTPUT);

  Serial.begin(9600);
}

void loop() {

  static int state = 0;
  //static String msg = "";
  static int msg = 0;
  
  // put main code here, to run repeatedly:

  if (Serial.available() > 0) {
    msg = Serial.parseInt();
    
    Serial.println(msg);
    state = msg;

    if (Serial.read() == '\n') {
      if (state) {
        Serial.println("SET GPIO ON");
      } else {
        Serial.println("SET GPIO OFF");
      }
    }
  }

  if (state){
    digitalWrite(LEDR, HIGH);       // Red LED on
    digitalWrite(gpio_out, HIGH);
  } else {
    digitalWrite(LEDR, LOW);        // Red LED off
    digitalWrite(gpio_out, LOW);
  }
  
  delay(100);
}