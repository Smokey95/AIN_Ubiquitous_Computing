/**
 * @file      main.ino
 * @brief     Main file for the project
 * @author    Smokey95
 * @authors   This script contains code published from Adafruit Industries by:
 *            Limor Fried/Ladyada
 * @note      Adafruit invests time and resources providing open source code,
              please support Adafruit and open-source hardware by purchasing
              products from Adafruit!
 * @copyright This code is licensed under the MIT License while the Adafruit
              fingerprint library is licensed under the BSD License
*/

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <WiFiNINA.h>
#include <Arduino_LSM6DSOX.h>
#include <PubSubClient.h>
#include <Adafruit_Fingerprint.h>

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

const int PIN_FINGER_REQ  = 8;              /**< GPIO pin for finger print sensor request */

/* WIFI Connection Details */
#define MAX_WIFI_CONNECTION_ATTEMPTS 5      /**< Attempts to connect to WiFi */
const char ssid[]   = WIFI_SSID;            /**< network SSID (name) */
const char pass[]   = WIFI_PASS;            /**< network password */

/* Finger Print Sensor */
#define mySerial Serial1                    /**< Hardware serial port */

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
int status          = WL_IDLE_STATUS;       /**< WiFi radio's status */
WiFiSSLClient wifiClient;
PubSubClient mqttClient(wifiClient);

/* Finger Print Sensor */
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial); /**< Finger print sensor */
uint8_t finger_id = 0;                        /**< Finger print ID */

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

  // Initialize door lock
  init_door_lock();

  // Initialize finger print sensor
  init_finger_print_sensor();

  // Show that setup is finished
  setColor(0, 255, 0);
  delay(3000);
  setColor(0, 0, 0);
}

/*******************************************************************************
 * Main
 ******************************************************************************/

void loop() {

  static int loop_counter = 100;

  mqttClient.loop();

  // Check door status
  update_door_main_status();
  update_door_letter_status();

  // Check for finger print
  if (get_finger_print_trigger()){
    if (verify_finger()){
      unlock_door();
    }
  }

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
        mqttClient.subscribe(TP_FINGER_SCAN);
        Serial.println("Subscribed to topic: " + String(TP_FINGER_SCAN));
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

/**
 * @brief   Initializes the finger print sensor
 * @author  Limor Fried/Ladyada for Adafruit Industries.
*/
void init_finger_print_sensor(){
  // set the data rate for the sensor serial port
  Serial.println("Initializing finger print sensor...");
  finger.begin(57600);
  delay(5);
  if (finger.verifyPassword()) {
    Serial.println("Found fingerprint sensor!");
    finger.LEDcontrol(false);
    // Init finger print hardware trigger too
    pinMode(PIN_FINGER_REQ, INPUT_PULLDOWN);
  } else {
    Serial.println("Did not find fingerprint sensor!");
    Serial.println("Restart system and try again!");
    Serial.println("If problem persists, contact support :)");
    error_state();
  }
}

bool get_finger_print_trigger(){
  return digitalRead(PIN_FINGER_REQ);
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
  } else if (incomingMessage == "SCAN"){
    if (verify_finger()){
      unlock_door();
    }
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
    send_main_door_status();
    if (door_main_open){
      setColor(0, 0, 0);
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

/*******************************************************************************
 * Lock Functions
 ******************************************************************************/

void unlock_door(){
  // Unlock door
  digitalWrite(PIN_DOOR_LOCK, HIGH);
  Serial.println("Door unlocked!");

  // Show that door is unlocked
  setColor(0, 255, 0);

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

  // Show that door is locked
  setColor(0, 0, 0);
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

/*******************************************************************************
 * Finger Print Sensor Functions
 ******************************************************************************/

/**
 * @brief   Prints the finger print sensor info to the serial
 * @note    Blocking function (infinite loop)
 * @author  Limor Fried/Ladyada for Adafruit Industries.
*/
void print_FingerPrint_Sensor_Info(){
  Serial.println(F("Reading sensor parameters"));
  finger.getParameters();
  Serial.print(F("Status: 0x")); Serial.println(finger.status_reg, HEX);
  Serial.print(F("Sys ID: 0x")); Serial.println(finger.system_id, HEX);
  Serial.print(F("Capacity: ")); Serial.println(finger.capacity);
  Serial.print(F("Security level: ")); Serial.println(finger.security_level);
  Serial.print(F("Device address: ")); Serial.println(finger.device_addr, HEX);
  Serial.print(F("Packet len: ")); Serial.println(finger.packet_len);
  Serial.print(F("Baud rate: ")); Serial.println(finger.baud_rate);

  finger.getTemplateCount();

  if (finger.templateCount == 0) {
    Serial.print("Sensor doesn't contain any fingerprint data. Please contact support :)");
  }
  else {
    Serial.println("Waiting for valid finger...");
    Serial.print("Sensor contains "); 
    Serial.print(finger.templateCount); 
    Serial.println(" templates");
  }
}

/**
 * @brief   Gets the finger print ID from the sensor
 * @return  Finger print ID or -1 if no finger print was found
 * @author  Fried/Ladyada for Adafruit Industries.
*/
uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      break;
    case FINGERPRINT_NOFINGER:
      Serial.println("No finger detected");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK success!

  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK converted!
  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
    Serial.println("Found a print match!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_NOTFOUND) {
    Serial.println("Did not find a match");
    return p;
  } else {
    Serial.println("Unknown error");
    return p;
  }

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);

  return finger.fingerID;
}

/**
 * @brief   Gets the finger print ID from the sensor
 * @return  Finger print ID or -1 if no finger print was found
 * @author  Limor Fried/Ladyada for Adafruit Industries.
*/
int getFingerprintIDez() {
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)  return -1;

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);
  return finger.fingerID;
}

uint8_t scan_finger(){
  finger.LEDcontrol(true);
  delay(500);
  while(true){
    int id = getFingerprintIDez();
    if (id != -1){
      Serial.print("Finger print detected! ID: ");
      Serial.println(id);
      finger.LEDcontrol(false);
      return id;
    }
    delay(50);
  }
}

bool verify_finger(){
  finger_id = scan_finger();
  if (finger_id == 1 || finger_id == 2){
    Serial.println("Finger print detected!");
    return true;
  } else {
    Serial.println("Finger print not detected!");
    return false;
  }
}