#include "painlessMesh.h"
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <map>

#define MESH_PREFIX "Fire_Gas_Detector"
#define MESH_PASSWORD "saveslifefromfire"
#define MESH_PORT 5555

#define WIFI_SSID "<WiFi_Name>"
#define WIFI_PASSWORD "<Wifi_Password>"

// WiFi Credentials
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

// MQTT Broker
const char* mqtt_server = "127.0.0.1";  
const int mqtt_port = 1883;

// MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);

std::map<short, std::pair<float, float>> nodeData;

const uint8_t room = 0;  //Assign room number for the room you want the sensor to be installed

const uint8_t firepin = D5;  // Fire sensor pin
const uint8_t gaspin = D2;   // Gas sensor pin
const uint8_t buzzer = D6;   // Buzzer output pin
short gas_threshold = 500;   // Set the gas threshold

bool buzzerActive = false;  // Buzzer reset flag

Scheduler userScheduler;
painlessMesh mesh;

uint32_t nodeId;  // Stores Node ID

void receivedCallback(uint32_t from, String &msg);
void readSensor();
void checkConnections();

Task taskReadSensors(TASK_MILLISECOND * 100, TASK_FOREVER, &readSensor);
Task taskCheckConnections(TASK_SECOND * 5, TASK_FOREVER, &checkConnections);

void checkConnections() {
  auto nodes = mesh.getNodeList();  // Get a list of connected nodes
  if (nodes.size() == 0) {
    Serial.println("No active nodes found in the mesh network.");
  } else {
    Serial.printf("Active nodes: %u\n", nodes.size());
  }
}

void readSensor() {
  if (manualReset) {
    digitalWrite(outpin, LOW);
  } else {
    if (digitalRead(firepin) == HIGH || digitalRead(gaspin) == HIGH) {
      digitalWrite(outpin, HIGH);
    } else {
      digitalWrite(outpin, LOW);
    }
  }
}

void receivedCallback(uint32_t from, String &msg) {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, msg);

  if (error) {
    Serial.print("Failed to parse JSON: ");
    Serial.println(error.c_str());
    return;
  }
  
  if (doc.containsKey("node") && doc.containsKey("fire") && doc.containsKey("gas")){
    short node = doc["node"];
    bool fire = doc["fire"];
    short gas = doc["gas"];

    Serial.println("Node %d, Fire: %d, gas: %d", node, fire, gas);
    
    if ((gas > gas_threshold || fire == true) && buzzerActive == false){
      digitalWrite(buzzer, HIGH);
      buzzerActive = true;
    }
  }
    
  } else {
    Serial.println("Problem with JSON Data...")
  }
}

void reconnect() {
  // Loop until connected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {  // Set a unique client ID for multiple gateways
      Serial.println("connected");
      client.subscribe("reset");  // Subscribe to your topics
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }

void setup(){
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
