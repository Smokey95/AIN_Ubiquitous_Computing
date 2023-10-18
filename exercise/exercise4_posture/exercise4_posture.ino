#include <WiFiNINA.h>
#include <Arduino_LSM6DSOX.h>   // Library required to access data froom LSM6DSOX
#include <MadgwickAHRS.h>       // Library 

Madgwick filter;

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

  // start the filer
  filter.begin(104);
}

void loop() {

  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;  

  // Check temperature
  int temp = check_temp(false);

  // get acceleration and gyroscope data
  if(IMU.accelerationAvailable() && 
     IMU.gyroscopeAvailable() &&
     temp <= 36){

    blink_green();
    
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

    Serial.print("Orientation: [");
    Serial.print(heading);
    Serial.print(" Yaw] [");
    Serial.print(pitch);
    Serial.print(" Pitch] [");
    Serial.print(roll);
    Serial.println(" Roll]");
  } else {
    blink_red();
  }
  
  delay(1000);  // measure every second
}

int check_temp(bool print_temperature) {
  // Check if temperature is available
  if (IMU.temperatureAvailable())
  {
    int temperature_deg = 0;
    IMU.readTemperature(temperature_deg);

    // Offset of ~30%
    int temperature_normalized = temperature_deg / 1.3;

    if(print_temperature){
      print_temp_serial(temperature_normalized);
    }

    return temperature_normalized;
  }
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
