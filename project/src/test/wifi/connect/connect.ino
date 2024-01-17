/**
 * @file connect.ino
 * @brief Test file for testing WiFi connection
*/

#include <SPI.h>
#include <WiFiNINA.h>

#include "credentials.h"

const int led_out  = 3;

char ssid[]   = WIFI_SSID;        // your network SSID (name)
char pass[]   = WIFI_PASS;        // your network password (use for WPA, or use as key for WEP)
int status    = WL_IDLE_STATUS;     // the WiFi radio's status

void setup(){

  pinMode(led_out, OUTPUT);
  blink_once();

  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  blink_once();

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
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    blink_connection();
  }

  // you're connected now, so print out the data:
  Serial.print("You're connected to the network");
  printCurrentNet();
  printWifiData();

  blink_success();
}

void loop() {
  // check the network connection once every 10 seconds:
  delay(10000);
  printCurrentNet();
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