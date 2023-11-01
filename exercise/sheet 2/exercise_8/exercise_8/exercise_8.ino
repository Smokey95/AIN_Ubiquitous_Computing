#include <WiFiNINA.h>
#include <Arduino_LSM6DSOX.h>
#include <MadgwickAHRS.h>

// WiFi credentials
char ssid[] = "yourNetwork";     //  your network SSID (name)
char pass[] = "secretPassword";  // your network password

// Madgwick filter
Madgwick filter;

void setup() {
  // Initialize serial communication and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Connect to Wi-Fi
  //while (WiFi.status() != WL_CONNECTED) {
  //  WiFi.begin(ssid, pass);
  //  delay(10000); // Attempt to connect to Wi-Fi every 10 seconds
  //}

  // Initialize the LSM6DSOX
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  filter.begin(104.00);

  // Initalize LEDs
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  digitalWrite(LEDR, LOW);    // Red
  digitalWrite(LEDB, LOW);    // Blue
  digitalWrite(LEDG, LOW);    // Green
}

void loop() {
  // define accelerometer & gyroscop data
  float ax, ay, az;
  float gx, gy, gz;

  static int counter = 0;

  // True = Outdor False = Indor
  static bool state;

  static float temp = 0;

  // check if data is available every loop
  if(IMU.accelerationAvailable() && 
     IMU.gyroscopeAvailable()) {

    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    // Update the filter with the latest data
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // Check the filter's roll for door movement
    if (filter.getRoll() > 5){
      state = true;
      digitalWrite(LEDR, HIGH); // Turn on LED if door is opened
    } else if (filter.getRoll() < -5) {
      state = false;
      digitalWrite(LEDR, HIGH); // Turn on LED if door is opened
    } else {
      digitalWrite(LEDR, LOW); // Turn off LED if door is closed
    }
  }

  if(!(counter % 100)){
    temp = get_temp();
    send_data(temp, state);
  }

  counter > 100 ? counter = 1 : counter++;
  delay(100);
}

float get_temp() {
  // Check if temperature is available
  if (IMU.temperatureAvailable())
  {
    int temperature_deg = 0;
    IMU.readTemperature(temperature_deg);

    // Offset of ~30%
    float temperature_normalized = temperature_deg / 1.3;

    return temperature_normalized;
  }
}

void send_data(float temp, bool in_out){
  // Construct JSON-style string
  String jsonString = "{\"temperature\": " + String(temp) + ", \"state\": \"" + (in_out ? "outdoor" : "indoor") + "\"}";
  Serial.println(jsonString);
}

