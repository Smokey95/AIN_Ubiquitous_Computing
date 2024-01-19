/**
 * @file connect.ino
 * @brief Test file for testing WiFi connection
*/

#include <SPI.h>
#include <WiFiNINA.h>
#include <Arduino_LSM6DSOX.h>   // Library required to access data froom LSM6DSOX

#include <PubSubClient.h>

#include "credentials.h"

#define MAX_WIFI_CONNECTION_ATTEMPTS 2

const int led_out  = 3;

/* WIFI Connection Details */
const char ssid[]   = WIFI_SSID;        // your network SSID (name)
const char pass[]   = WIFI_PASS;        // your network password (use for WPA, or use as key for WEP)
int status    = WL_IDLE_STATUS;     // the WiFi radio's status

/* MQTT Connection Details */
const char* mqtt_server   = MQTT_SERVER;
const char* mqtt_username = MQTT_USERNAME;
const char* mqtt_pw       = MQTT_PW;
const char* mqtt_port     = MQTT_PORT;

/* WiFi Connection Details */
WiFiClient ethClient;

/* MQTT Client Initialization using WiFi Client */
PubSubClient client(ethClient);

/* MQTT Message Buffer */
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];

const char* server = MQTT_SERVER;

void setup(){

  pinMode(led_out, OUTPUT);
  blink_once();

  setup_Serial();
  blink_once();

  setup_wifi_connection();
  blink_success();
  delay(2000);

  client.setServer(server, MQTT_PORT);

  // Initialize LSM6DSOX librarys
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  // check the network connection once every 10 seconds:
  delay(1000);
  //printCurrentNet();

  print_temp_serial(get_temperature());
}


void reconnect() {
  // Loop until we're reconnected
  Serial.print("Attempting MQTT connection...");
  // Attempt to connect
  if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PW)) {
    Serial.println("connected");
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 5 seconds");
    // Wait 5 seconds before retrying
    delay(5000);
  }
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
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void setup_wifi_connection(){
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    blink_error();
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }
  blink_once();

  // attempt to connect to Wifi network:
  int attempts = 0;
  while (status != WL_CONNECTED && attempts < MAX_WIFI_CONNECTION_ATTEMPTS) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    blink_connection();

    attempts++;
  }

  if (attempts >= MAX_WIFI_CONNECTION_ATTEMPTS){
    blink_error();
  }

  // you're connected now, so print out the data:
  Serial.print("You're connected to the network");
  printCurrentNet();
  printWifiData();
}

void blink_error(){
  while(true){
    digitalWrite(led_out, HIGH);
    delay(100);
    digitalWrite(led_out, LOW);
    delay(100);
  }
}

void blink_once(){
  digitalWrite(led_out, HIGH);
  delay(100);
  digitalWrite(led_out, LOW);
  delay(100);
}

void blink_connection(){
  for (int i = 0; i < 10; i++){
    digitalWrite(led_out, HIGH);
    delay(500);
    digitalWrite(led_out, LOW);
    delay(500);
  }
}

void blink_success(){
  digitalWrite(led_out, HIGH);
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