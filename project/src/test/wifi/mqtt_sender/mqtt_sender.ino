/**
 * @file connect.ino
 * @brief Test file for testing WiFi connection
*/

#include <SPI.h>
#include <WiFiNINA.h>
#include <Arduino_LSM6DSOX.h>   // Library required to access data froom LSM6DSOX

#include <PubSubClient.h>

#include "credentials.h"
#include "topics.h"

/* LED Pin */
const int PIN_RED_LED     = 2;
const int PIN_BLUE_LED    = 3;
const int PIN_GREEN_LED   = 4;

/* WIFI Connection Details */
#define MAX_WIFI_CONNECTION_ATTEMPTS 5  // Number of attempts to connect to WiFi
const char ssid[]   = WIFI_SSID;        // your network SSID (name)
const char pass[]   = WIFI_PASS;        // your network password (use for WPA, or use as key for WEP)
int status          = WL_IDLE_STATUS;   // the WiFi radio's status

/* MQTT Connection Details */
const char* mqtt_server   = MQTT_SERVER;
const char* mqtt_username = MQTT_USERNAME;
const char* mqtt_pw       = MQTT_PW;
const char* client_id     = MQTT_CLIENT_ID;
const int   mqtt_port     = MQTT_PORT;

/* WiFi Client Defintion */
WiFiSSLClient wifiClient;
PubSubClient mqttClient(wifiClient);

/* MQTT Message Buffer */
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];

void setup(){
  // Initialize LED
  pinMode(PIN_RED_LED, OUTPUT);
  pinMode(PIN_BLUE_LED, OUTPUT);
  pinMode(PIN_GREEN_LED, OUTPUT);
  blink_once_blue();

  // Initialize Serial Interface
  setup_Serial();

  // Initialize IMU Sensor
  setup_IMU();

  // Initialize WiFi Connection
  setup_wifi_connection();

  // Initialize MQTT Connection
  setup_MQTT();  
}

void loop() {

  static int loop_counter = 0;

  mqttClient.loop();

  // Read temperature from IMU
  int temperature_deg = get_temperature();

  if (loop_counter % 100 == 0){
    // Print temperature to serial
    print_temp_serial(temperature_deg);

    // Send data to MQTT server
    publish_message(TP_ALIVE, String("true"));
    //publish_message(TP_TEMP, String(temperature_deg));

    // Reset loop counter
    loop_counter = 0;
  }

  delay(100);

  loop_counter++;
}

void setup_MQTT(){
  Serial.println("Setting up MQTT...");
  blink_once_blue();

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(callback);
  blink_once_blue();

  for (int i = 0; i < 5; i++){
    blink_once_blue();
    Serial.println("Connecting to MQTT server [" + String(i) + "/5]...");
    if (!mqttClient.connected()) {
      if (mqttClient.connect(client_id, mqtt_username, mqtt_pw)) {
        Serial.println("MQTT connected!");
        mqttClient.subscribe(TP_LED);
        Serial.println("Subscribed to topic: " + String(TP_LED));
        break;
      } else {
        Serial.print("failed, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        blink_connection(5);
      }
    }
  }

  if (!mqttClient.connected()) {
    Serial.println("MQTT connection failed!");
    blink_error();
  }
}

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
  } else {
    setColor(0, 0, 0);
  }
}

void publish_message(const char* topic, String payload){
  if (!mqttClient.connected()) {
    setup_MQTT();
  }
  mqttClient.publish(topic, payload.c_str(), true);
  Serial.println("Message published to MQTT server: " + payload);
  //blink_data_send();
}

void setup_IMU(){
  // Initialize LSM6DSOX library's
  Serial.println("Initializing IMU...");
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    blink_error();
  }
  Serial.println("IMU initialized!");
  blink_once_blue();
}

void printWifiData() {
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);
}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}

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

void setup_Serial(){
  Serial.begin(9600);
  //while (!Serial) {
  //  ; // wait for serial port to connect. Needed for native USB port only
  //}
  blink_once_blue();
}

void setup_wifi_connection(){
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    blink_error();
  }
  blink_once_blue();

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }
  blink_once_blue();

  // attempt to connect to Wifi network:
  int attempts = 0;
  while (status != WL_CONNECTED && attempts < MAX_WIFI_CONNECTION_ATTEMPTS) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    blink_connection(10);

    attempts++;
  }

  if (attempts >= MAX_WIFI_CONNECTION_ATTEMPTS){
    blink_error();
  }

  // you're connected now, so print out the data:
  Serial.print("You're connected to the network");
  printCurrentNet();
  printWifiData();

  // Give the hardware a breather
  delay(1000);
}

void blink_error(){
  while(true){
    setColor(255, 0, 0);
    delay(100);
    setColor(0, 0, 0);
    delay(100);
  }
}

void blink_once_blue(){
  setColor(0, 0, 255);
  delay(100);
  setColor(0, 0, 0);
  delay(100);
}

void blink_data_send(){
  setColor(0, 255, 0);
  delay(40);
  setColor(0, 0, 0);
}

void blink_connection(int time_sec){
  for (int i = 0; i < time_sec; i++){
    setColor(0, 0, 255);
    delay(500);
    setColor(0, 0, 0);
    delay(500);
  }
}

void print_temp_serial(int temp){
  Serial.print("LSM6DSOX Temperature = ");
  Serial.print(temp);
  Serial.println(" °C");
}

int get_temperature(){
  int temperature_deg = 0;
  if (IMU.temperatureAvailable())
  {
    IMU.readTemperature(temperature_deg);
  }
  return temperature_deg;
}

void setColor(int R, int G, int B) {
  analogWrite(PIN_RED_LED, 255 - R);
  analogWrite(PIN_GREEN_LED, 255 - G);
  analogWrite(PIN_BLUE_LED, 255 - B);
}