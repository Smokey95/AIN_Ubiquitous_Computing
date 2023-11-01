#include <WiFiNINA.h>
#include <Arduino_LSM6DSOX.h>   // Library required to access data froom LSM6DSOX

void setup() {

  // Initalize LEDs
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  // Initialize LSM6DSOX librarys
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (IMU.temperatureAvailable())
  {
    int temperature_deg = 0;
    IMU.readTemperature(temperature_deg);

    // Offset of ~30%
    float temperature_normalized = temperature_deg / 1.3;

    // Create a JSON-formatted string
    String jsonString = "{\"temp\": " + String(temperature_normalized) + "}";

    // Send JSON-formatted string over serial
    Serial.println(jsonString);

    if(temperature_normalized > 36) {
      blink_red();
    } else if(temperature_normalized >= 20 && 
              temperature_normalized <= 36) {
      blink_green();
    } else if (temperature_normalized < 20) {
      blink_blue();
    }
    
  }
  delay(5000);  // measure every second
}

void print_temp_serial(int temp){
  Serial.print("LSM6DSOX Temperature = ");
  Serial.print(temp);
  Serial.println(" °C");
}

void blink_red(){
  digitalWrite(LEDR, HIGH);   // Red
  digitalWrite(LEDB, LOW);    // Blue
  digitalWrite(LEDG, LOW);    // Green
}

void blink_blue(){  
  digitalWrite(LEDR, LOW);    // Red
  digitalWrite(LEDB, HIGH);   // Blue
  digitalWrite(LEDG, LOW);    // Green
}

void blink_green(){
  digitalWrite(LEDR, LOW);    // Red
  digitalWrite(LEDB, LOW);    // Blue
  digitalWrite(LEDG, HIGH);   // Green
}
