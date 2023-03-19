#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include "secrets.h"
// #define WIFI_SSID 
// #define WIFI_PASS
// #define IO_USERNAME
// #define IO_KEY 
// #define MQTT_IN_FEED 
// #define MQTT_OUT_FEED 

#define RELAY_CTRL_PIN 27

WiFiClient espClient;
PubSubClient client(espClient);

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if((char)payload[0]=='1'){ // short press
    digitalWrite(LED_BUILTIN, HIGH);

    digitalWrite(RELAY_CTRL_PIN, HIGH);
    delay(500);
    digitalWrite(RELAY_CTRL_PIN, LOW);

    digitalWrite(LED_BUILTIN, LOW);
  }
  else if((char)payload[0]=='2'){ // long press
    digitalWrite(LED_BUILTIN, HIGH);

    digitalWrite(RELAY_CTRL_PIN, HIGH);
    delay(5000);
    digitalWrite(RELAY_CTRL_PIN, LOW);

    digitalWrite(LED_BUILTIN, LOW);
  }
}

void connectToMqtt() {
  client.setServer("io.adafruit.com", 1883);
  client.setCallback(callback);
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClient", IO_USERNAME, IO_KEY)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(MQTT_OUT_FEED,":)");
      // ... and resubscribe
      client.subscribe(MQTT_IN_FEED);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void connectToWifi(){
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while(WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void reconnectToWifiAndMqtt(){
  // Drop existing connections
  client.disconnect();
  WiFi.disconnect();

  // Connect to WIFI
  connectToWifi();

  // Connect to AdafruitIO MQTT broker
  connectToMqtt();
}

void setup(){
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RELAY_CTRL_PIN, OUTPUT);

  reconnectToWifiAndMqtt();  
}

void loop(){
  // Reconnect WiFI and MQTT if it is down
  if(WiFi.status() != WL_CONNECTED){
    reconnectToWifiAndMqtt();
  }
  // Reconnect to just MQTT if it is down
  else if(!client.connected()){
    connectToMqtt();
  }
  // Reconnect wifi+mqtt if wifi is down
  else {
    client.loop();
  }
}