#include "painlessMesh.h"
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

#define MESH_PREFIX "Fire_Gas_Detector"
#define MESH_PASSWORD "saveslifefromfire"
#define MESH_PORT 5555

// WiFi Credentials
const char* ssid = "GOD_GIFTED_4G";
const char* password = "ssgrp1234";

// MQTT Broker
const char* mqtt_server = "192.168.29.192";  
const int mqtt_port = 1884;

// MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);

const uint8_t room = 0;  //Assign room number for the room you want the sensor to be installed

const uint8_t firepin = D5;  // Fire sensor pin
const uint8_t gaspin = A0;   // Gas sensor pin
const uint8_t buzzer = D6;   // Buzzer output pin
short gas_threshold = 500;   // Set the gas threshold

unsigned long endTime = 0;   // For buzzer
const short interval = 500;   // Set buzzer interval
bool buzzerActive = false;   // Buzzer reset flag

Scheduler userScheduler;
painlessMesh mesh;

uint32_t nodeId;  // Stores Node ID

void receivedCallback(uint32_t from, String &msg);
void readSensor();
void checkConnections();

Task taskReadSensors(TASK_MILLISECOND * 500, TASK_FOREVER, []() {
    readSensor();
    taskReadSensors.setInterval(random(TASK_MILLISECOND * 450, TASK_MILLISECOND * 550));  // Adjust interval dynamically
});
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
  short gas = analogRead(gaspin);
  bool fire = digitalRead(firepin);

  //Triggers alarm
  if((fire || gas > gas_threshold) && buzzerActive == false){
    buzzerActive = true;
  }

  // Construct MQTT topic for gateway node
  String gasTopic = "sensor_data/Node_0/gas";
  String fireTopic = "sensor_data/Node_0/fire";

  //Publishing data
  client.publish(gasTopic.c_str(), String(gas).c_str(), true);
  client.publish(fireTopic.c_str(), fire ? "1" : "0", true);

  Serial.printf("Published: Gas = %d, Fire = %d\n", gas, fire);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT Callback received: ");

  String msg;
  for(int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.println(msg);

  if(String(topic) == "reset") {
    if (msg == "1"){
        if (mesh.sendBroadcast("reset")) {
          Serial.println("Reset message broadcasted successfully.");
        } else {
          Serial.println("Reset message broadcast failed.");
        }
        buzzerActive = false;
    }
  }
}


void receivedCallback(uint32_t from, String &msg) {
  Serial.print("Received message from node ");
  Serial.print(from);
  Serial.print(":");
  Serial.print(msg);
  
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

    Serial.printf("Node %d, Fire: %d, gas: %d\n", node, fire, gas);

    //Activates buzzer
    if((fire || gas > gas_threshold) && buzzerActive == false){
    buzzerActive = true;
    }

    // Construct MQTT topic for this specific node
    String gasTopic = "sensor_data/Node_" + String(node) + "/gas";
    String fireTopic = "sensor_data/Node_" + String(node) + "/fire";

    // Publish data to MQTT broker
    if (client.publish(gasTopic.c_str(), String(gas).c_str(), true)) {
      Serial.println("Published gas data successfully");
    } else {
      Serial.println("Failed to publish gas data");
    }

    if (client.publish(fireTopic.c_str(), String(fire).c_str(), true)) {
      Serial.println("Published fire data successfully");
    } else {
      Serial.println("Failed to publish fire data");
    }
  }    
  else {
    Serial.println("JSON data missing required keys (node, fire, gas)");
  }
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("New Connection: NodeID = %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),offset);
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
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(firepin, INPUT);
  pinMode(gaspin, INPUT);
  pinMode(buzzer, OUTPUT);

  // Connect to WiFi with timeout
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected!");

  Serial.println(WiFi.localIP());
  Serial.println(mqtt_server);

  // Initialize mesh network
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  
  //Setup MQTT Client
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);  

  userScheduler.addTask(taskCheckConnections);
  userScheduler.addTask(taskReadSensors);
  taskCheckConnections.enable();
  taskReadSensors.enable();

}

void loop(){
  if (!client.connected()) {
    reconnect();
  }
  mesh.update();
  client.loop();
  
  //Part to trigger buzzer
  if (buzzerActive) {
    if (millis() - endTime >= interval){
      endTime = millis();
      digitalWrite(buzzer, !digitalRead(buzzer));
    }
  } else {
    digitalWrite(buzzer, LOW);
  }
}
