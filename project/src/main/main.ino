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
#include <PubSubClient.h>

#include "lib\credentials.h"
#include "lib\topics.h"

/*******************************************************************************
 * Constants
 ******************************************************************************/
const int PIN_RED_LED     = 2;              /**< GPIO pin for red LED */
const int PIN_GREEN_LED   = 3;              /**< GPIO pin for green LED */
const int PIN_BLUE_LED    = 4;              /**< GPIO pin for blue LED */

const int PIN_DOOR_MAIN   = 5;              /**< GPIO pin for main door sensor */
const int PIN_DOOR_LETTER = 6;              /**< GPIO pin for letter box door sensor */

const int PIN_DOOR_LOCK   = 7;              /**< GPIO pin for door lock */

/* WIFI Connection Details */
#define MAX_WIFI_CONNECTION_ATTEMPTS 5      /**< Attempts to connect to WiFi */
const char ssid[]   = WIFI_SSID;            /**< network SSID (name) */
const char pass[]   = WIFI_PASS;            /**< network password */
int status          = WL_IDLE_STATUS;       /**< WiFi radio's status */

/* MQTT Connection Details */
const char* mqtt_server   = MQTT_SERVER;    /**< MQTT server address */
const char* mqtt_username = MQTT_USERNAME;  /**< MQTT username */
const char* mqtt_pw       = MQTT_PW;        /**< MQTT password */
const char* client_id     = MQTT_CLIENT_ID; /**< MQTT client ID */
const int   mqtt_port     = MQTT_PORT;      /**< MQTT port */


/*******************************************************************************
 * Global Variables
 ******************************************************************************/

/* WiFi Client Defintion */
WiFiSSLClient wifiClient;
PubSubClient mqttClient(wifiClient);

/* Door Status */
bool door_main_open = false;                /**< Status of main door */
bool door_main_prev = false;                /**< Previous status of main door */
bool door_letter_open = false;              /**< Status of letter box door */
bool door_letter_prev = false;              /**< Previous status of letter box door */

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

  // Initialize MQTT connection
  init_MQTT();

  // Show that general setup is finished
  blink_x_times(3, 0, 255, 0, 1000);

  // Initialize door sensors
  init_DoorSensors();
}

/*******************************************************************************
 * Main
 ******************************************************************************/

void loop() {

  static int loop_counter = 10;

  mqttClient.loop();

  // Check door status
  update_door_main_status();
  update_door_letter_status();

  if (loop_counter % 100 == 0){
    send_alive();
    loop_counter = 0;
  }

  loop_counter++;
  delay(100);
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

/**
 * @brief   Initializes the door sensors
 * @note    One sensor for the main door and one for the letter box
*/
void init_DoorSensors(){
  // Initialize door sensors
  pinMode(PIN_DOOR_MAIN, INPUT_PULLUP);
  pinMode(PIN_DOOR_LETTER, INPUT_PULLUP);

  // Check door status
  update_door_main_status();
  update_door_letter_status();

  // Print door status
  print_door_status();

  // Show door status
  show_door_status();
}

void init_door_lock(){
  pinMode(PIN_DOOR_LOCK, OUTPUT);
}


/**
 * @brief   Initializes the WiFi connection
*/
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

/**
 * @brief   Initializes the MQTT connection
*/
void init_MQTT(){
  Serial.println("Setting up MQTT...");
  blink_x_times(1, 0, 0, 255, 500);

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(callback);
  blink_x_times(1, 0, 0, 255, 500);

  for (int i = 0; i < 5; i++){
    Serial.println("Connecting to MQTT server [" + String(i + 1) + "/5]...");
    if (!mqttClient.connected()) {
      if (mqttClient.connect(client_id, mqtt_username, mqtt_pw)) {
        Serial.println("MQTT connected!");
        mqttClient.subscribe(TP_LED);
        Serial.println("Subscribed to topic: " + String(TP_LED));
        mqttClient.subscribe(TP_DOOR_OPEN);
        Serial.println("Subscribed to topic: " + String(TP_DOOR_OPEN));
        break;
      } else {
        Serial.print("failed, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        blink_x_times(5, 0, 0, 255, 1000);
      }
    }
  }

  if (!mqttClient.connected()) {
    Serial.println("MQTT connection failed!");
    error_state();
  }
}

/*******************************************************************************
 * MQTT Functions
 ******************************************************************************/

/**
 * @brief   Callback function for MQTT
*/
void callback(char* topic, byte* payload, unsigned int length) {
  String incomingMessage = "";
  for (int i = 0; i < length; i++) {
    incomingMessage += (char)payload[i];
  }
  incomingMessage.trim();
  
  Serial.println("Message received from MQTT server: " + incomingMessage);
  if (incomingMessage == "1"){
    setColor(255, 0, 0);
  } else if (incomingMessage == "2"){
    setColor(0, 255, 0);
  } else if (incomingMessage == "3"){
    setColor(0, 0, 255);
  } else if (incomingMessage == "OPEN"){
    unlock_door();
  } else {
    setColor(0, 0, 0);
  }
}

void publish_message(const char* topic, String payload){
  if (!mqttClient.connected()) {
    Serial.print("MQTT connection lost! ");
    Serial.println("Attempting to reconnect to MQTT server");
    init_MQTT();
  }

  mqttClient.publish(topic, payload.c_str(), true);
  Serial.println("Message published to MQTT server: " + payload);
}

/**
 * @brief   Send an alive message to the MQTT server
*/
void send_alive(){
  publish_message(TP_ALIVE, "true");
}

void send_main_door_status(){
  publish_message(TP_DOOR_MAIN, String(door_main_open));
}

void send_letter_door_status(){
  publish_message(TP_DOOR_LETTER, String(door_letter_open));
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
 * Door Functions
 ******************************************************************************/

void update_door_main_status(){
  door_main_open = digitalRead(PIN_DOOR_MAIN);
  if (door_main_open != door_main_prev){
    door_main_prev = door_main_open;
    show_door_status();
    send_main_door_status();
    if (door_main_open){
      Serial.println("Main door opened!");
    } else {
      Serial.println("Main door closed!");
    }
  }
}

void update_door_letter_status(){
  door_letter_open = digitalRead(PIN_DOOR_LETTER);
  if (door_letter_open != door_letter_prev){
    door_letter_prev = door_letter_open;
    send_letter_door_status();
    if (door_letter_open){
      Serial.println("Letter box door opened!");
    } else {
      Serial.println("Letter box door closed!");
    }
  }
}

void print_door_status(){
  Serial.println("Main door open: " + String(door_main_open));
  Serial.println("Letter box door open: " + String(door_letter_open));
}

void show_door_status(){
  if (door_main_open){
    setColor(0, 255, 0);
  } else {
    setColor(0, 0, 0);
  }
}

/*******************************************************************************
 * Lock Functions
 ******************************************************************************/

void unlock_door(){
  // Unlock door
  digitalWrite(PIN_DOOR_LOCK, HIGH);
  Serial.println("Door unlocked!");

  // Wait 5 seconds for user to open the door
  int counter = 0;
  while (!door_main_open && counter < 50){
    update_door_main_status();
    delay(100);
    counter++;
  }

  // Lock door
  digitalWrite(PIN_DOOR_LOCK, LOW);
  Serial.println("Door locked!");
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
