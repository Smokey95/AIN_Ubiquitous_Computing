/**
 * @file    main.ino
 * @brief   Main file for the project
 * @author  Smokey95
*/

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <WiFiNINA.h>
#include <Arduino_LSM6DSOX.h>

#include "lib\credentials.h"

/*******************************************************************************
 * Constants
 ******************************************************************************/
const int PIN_RED_LED     = 2;        /**< GPIO pin for red LED */
const int PIN_GREEN_LED   = 3;        /**< GPIO pin for green LED */
const int PIN_BLUE_LED    = 4;        /**< GPIO pin for blue LED */

/* WIFI Connection Details */
#define MAX_WIFI_CONNECTION_ATTEMPTS 5  /**< Attempts to connect to WiFi */
const char ssid[]   = WIFI_SSID;        /**< network SSID (name) */
const char pass[]   = WIFI_PASS;        /**< network password */
int status          = WL_IDLE_STATUS;   /**< WiFi radio's status */

/*******************************************************************************
 * Setup
 ******************************************************************************/
void setup() {
  // Initialize internal LED
  init_LED();

  // Initialize serial communication
  init_serial();

  // Initialize IMU sensor
  init_IMU();

  // Initialize WiFi connection
  init_wifi_connection();
}

/*******************************************************************************
 * Main
 ******************************************************************************/

void loop() {

}

/*******************************************************************************
 * Init Functions
 ******************************************************************************/

/**
 * @brief   Initializes the internal LED
*/
void init_LED() {
  // Initialize LEDs
  pinMode(PIN_RED_LED, OUTPUT);
  pinMode(PIN_BLUE_LED, OUTPUT);
  pinMode(PIN_GREEN_LED, OUTPUT);

  // Blink once to indicate start
  setColor(255, 0, 0);
  delay(250);
  setColor(0, 255, 0);
  delay(250);
  setColor(0, 0, 255);
  delay(250);
  setColor(0, 0, 0);
}

/**
 * @brief   Initializes the serial communication
*/
void init_serial() {
  Serial.begin(9600);
  blink_x_times(2, 0, 0, 255, 500);
  // wait for 2 seconds for serial port to connect
  delay(2000);
  Serial.println("Serial initialized");
}

/**
 * @brief   Initializes IMU sensor and print status to serial
*/
void init_IMU(){
  // Initialize LSM6DSOX library's
  Serial.println("Initializing IMU...");
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    error_state();
  }
  Serial.println("IMU initialized!");
  blink_x_times(2, 0, 0, 255, 500);
  print_temp_serial(get_temperature());
}

void init_wifi_connection(){
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    error_state();
  }
  blink_x_times(1, 0, 0, 255, 500);

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("WARNING: Please upgrade the firmware");
  }
  blink_x_times(1, 0, 0, 255, 500);

  // attempt to connect to Wifi network:
  int attempts = 0;
  while (status != WL_CONNECTED && attempts < MAX_WIFI_CONNECTION_ATTEMPTS) {
    Serial.print("Attempting to connect to SSID [" + String(attempts + 1) + "/" + String(MAX_WIFI_CONNECTION_ATTEMPTS) + "]: ");
    Serial.println(ssid);
    
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    blink_x_times(10, 0, 0, 255, 1000);

    attempts++;
  }

  if (attempts >= MAX_WIFI_CONNECTION_ATTEMPTS){
    error_state();
  }

  // you're connected now, so print out the data:
  Serial.print("You're connected to the network");
  printCurrentNet();
  printWifiData();

  // Give the hardware a breather
  delay(1000);
}


/*******************************************************************************
 * LED Functions
 ******************************************************************************/

/**
 * @brief   Blinks the internal LED x times
 * @param   x   Number of times to blink
 * @param   R   Red value (0-255)
 * @param   G   Green value (0-255)
 * @param   B   Blue value (0-255)
 * @param   delay_ms   Duration of each blink in ms
 * @note    Blocking function for x * duration_ms
*/
void blink_x_times(int x, int R, int G, int B, int duration_ms){
  for (int i = 0; i < x; i++){
    setColor(R, G, B);
    delay(duration_ms / 2);
    setColor(0, 0, 0);
    delay(250 / 2);
  }
}

/**
 * @brief   Sets the color of the RGB LED
 * @param   R   Red value (0-255)
 * @param   G   Green value (0-255)
 * @param   B   Blue value (0-255)
*/
void setColor(int R, int G, int B) {
  analogWrite(PIN_RED_LED, 255 - R);
  analogWrite(PIN_GREEN_LED, 255 - G);
  analogWrite(PIN_BLUE_LED, 255 - B);
}

/*******************************************************************************
 * Error Functions
 ******************************************************************************/

/**
 * @brief   Blinks the internal LED in red to indicate an error
 * @note    Blocking function (infinite loop)
 * @note    Use only for critical non recoverable errors
*/
void error_state(){
  while(true){
    setColor(255, 0, 0);
    delay(500);
    setColor(0, 0, 0);
    delay(500);
  }
}

/*******************************************************************************
 * IMU Functions
 ******************************************************************************/

/**
 * @brief  Gets the temperature from the IMU
 * @return Temperature in °C or -273 if not available
*/
int get_temperature(){
  int temperature_deg = 0;
  if (IMU.temperatureAvailable())
  {
    IMU.readTemperature(temperature_deg);
    return temperature_deg;
  } else {
    return -273;
  }
}

/**
 * @brief   Prints the temperature to the serial
*/
void print_temp_serial(int temp){
  Serial.print("LSM6DSOX Temperature = ");
  Serial.print(temp);
  Serial.println(" °C");
}

/*******************************************************************************
 * WIFI Functions
 ******************************************************************************/

/**
 * @brief   Print the WiFi IP and MAC address to the serial
*/
void printWifiData() {
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);
}

/**
 * @brief   Print additional information about the WiFi connection to the serial
*/
void printCurrentNet() {
  // print SSID of the network
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print routers MAC address
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  // print received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print encryption type
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}

/**
 * @brief   Print the MAC address to the serial
*/
void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}
