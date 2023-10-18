// Exercise 0
const int ledPin =    LED_BUILTIN;  // The onboard LED pin
const int delayTime = 50;           // Delay time in milliseconds (1 second)

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  digitalWrite(ledPin, HIGH);  // Turn the LED on
  delay(delayTime);
  
  digitalWrite(ledPin, LOW);   // Turn the LED off
  delay(delayTime);
}