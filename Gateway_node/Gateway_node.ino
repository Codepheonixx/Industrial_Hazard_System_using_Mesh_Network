#include "painlessMesh.h"
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

#define MESH_PREFIX "Fire_Gas_Detector"
#define MESH_PASSWORD "saveslifefromfire"
#define MESH_PORT 5555

// WiFi Credentials
#define   STATION_SSID     "TEST_NETWORK_1"
#define   STATION_PASSWORD "12345678"

#define HOSTNAME "FireAlarm_Gateway"

const uint8_t room = 0;  //Assign room number for the room you want the sensor to be installed

const uint8_t firepin = D5;  // Fire sensor pin
const uint8_t gaspin = A0;   // Gas sensor pin
const uint8_t buzzer = D6;   // Buzzer output pin
short gas_threshold = 500;   // Set the gas threshold

unsigned long lastReconnectAttempt = 0;
unsigned long reconnectInterval = 5000; // Retry every 5 seconds

bool buzzerActive = false;
unsigned long buzzerStartTime = 0;
unsigned long lastBlinkTime = 0;
const unsigned long BUZZER_BLINK_INTERVAL = 500; // blink every 500ms
bool buzzerState = false; // tracks ON/OFF state of buzzer for blinking

IPAddress getlocalIP();

IPAddress myIP(0,0,0,0);
IPAddress mqttBroker(192, 168, 32, 64);

void receivedCallback(uint32_t from, String &msg);
void mqttCallback(char* topic, byte* payload, unsigned int length);

painlessMesh mesh;
WiFiClient wifiClient;
PubSubClient mqttClient(mqttBroker, 1884, mqttCallback, wifiClient); //Set MQTT

uint32_t nodeId;  // Stores Node ID

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("New Connection: NodeID = %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),offset);
}


void setup() {

  Serial.begin(115200);
  pinMode(firepin, INPUT);
  pinMode(gaspin, INPUT);
  pinMode(buzzer, OUTPUT);

  digitalWrite(firepin,LOW);
  digitalWrite(buzzer,LOW);


  // Initialize mesh network
  mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
  mesh.init( MESH_PREFIX, MESH_PASSWORD, MESH_PORT, WIFI_AP_STA, 6 );
  mesh.onReceive(&receivedCallback);
  mesh.stationManual(STATION_SSID, STATION_PASSWORD);

  WiFi.begin(STATION_SSID, STATION_PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  //WiFi.disconnect();

  mesh.setHostname(HOSTNAME);
  mesh.setRoot(true);
  mesh.setContainsRoot(true);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

}

void loop(){
  //Handles Network
  mesh.update();      // Handle mesh network
  mqttClient.loop();  // Process MQTT messages
  
  // Tracks IP change
  if(myIP != getlocalIP()){
    myIP = getlocalIP();
    Serial.println("My IP is " + myIP.toString());
  }

  // MQTT Check
  if (!mqttClient.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt >= reconnectInterval) {
      lastReconnectAttempt = now;
      if (reconnect()) {
        Serial.println("MQTT reconnected...");
        reconnectInterval = 5000;
      } else {
        reconnectInterval = min(reconnectInterval * 2, (unsigned long) 30000); // Exponential backoff (max 60s)
        Serial.print("Retrying in ");
        Serial.print(reconnectInterval / 1000);
        Serial.println("s...");
      }
    }
  }
  
  // Reads sensor every 1s
  static unsigned long lastRead = 0;
  if (millis() - lastRead >= 1000) {
    lastRead = millis();
    if (mqttClient.connected()) readSensor();
  }

  //Part to trigger buzzer
  if (buzzerActive) {
    unsigned long currentTime = millis();

    // Blink the buzzer every 500ms
    if (currentTime - lastBlinkTime >= BUZZER_BLINK_INTERVAL) {
      buzzerState = !buzzerState;
      digitalWrite(buzzer, buzzerState ? HIGH : LOW);
      lastBlinkTime = currentTime;
    }
  } else {
    digitalWrite(buzzer, LOW);
    buzzerState = false;
  }

}

void readSensor() {
  // Sensor Readings
  short gas = analogRead(gaspin);
  bool fire = digitalRead(firepin);

  // Alarm Trigger
  if ((fire || gas > gas_threshold) && !buzzerActive) {
    buzzerActive = true;
    buzzerStartTime = millis();
    Serial.println("Hazard detected !!");
  }

  // MQTT Publishing (optimized)
  char gasTopic[32], fireTopic[32];
  snprintf(gasTopic, sizeof(gasTopic), "sensor_data/Node_0/gas");
  snprintf(fireTopic, sizeof(fireTopic), "sensor_data/Node_0/fire");

  bool success = true;
  success &= mqttClient.publish(gasTopic, String(gas).c_str(), true);
  success &= mqttClient.publish(fireTopic, fire ? "1" : "0", true);

  if (!success) {
    Serial.printf("Publish failed (state=%d)\n", mqttClient.state());
  }
}

bool reconnect() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    mesh.stationManual(STATION_SSID, STATION_PASSWORD); // Re-init Wi-Fi
    Serial.print(WiFi.localIP());
  }

  char clientId[32];
  snprintf(clientId, sizeof(clientId), "ESP_Gateway-%08X", ESP.getChipId());
  
  if (mqttClient.connect(clientId)) {
    mqttClient.publish("testTopic","Gateway node ready!");
    mqttClient.subscribe("Reset");
    return true;
  }
  
  Serial.printf("MQTT failed (rc=%d)\n", mqttClient.state());
  return false;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {

  String msg;
  for(int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.printf("MQTT Callback: Topic=%s, Msg=%s\n", topic, msg.c_str());

  if(String(topic) == "Reset" && msg == "1") { 

    buzzerActive = false;
    Serial.println("Reset command received. Broadcasting...");
    if (mesh.sendBroadcast("reset")) {
      Serial.println("Broadcasted successfully.");
    } else {
      Serial.println("Broadcast failed.");
    }
    
  }
}


void receivedCallback(uint32_t from, String &msg) {
    Serial.printf("[Mesh] Msg from %u: %s\n", from, msg.c_str());

    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
        Serial.printf("JSON error: %s\n", error.c_str());
        return;
    }

    // Validate fields
    if (!doc.containsKey("node") || !doc.containsKey("fire") || !doc.containsKey("gas")) {
        Serial.println("Invalid JSON structure");
        return;
    }

    // Extract data
    uint8_t node = doc["node"];
    bool fire = doc["fire"];
    uint16_t gas = doc["gas"];

    // Alarm logic
    bool hazardDetected = fire || gas > gas_threshold;
    if (hazardDetected && !buzzerActive) {
        buzzerActive = true;
        buzzerStartTime = millis();
        Serial.println("! HAZARD DETECTED !");
    }

    // MQTT publishing
    char topic[32];
    snprintf(topic, sizeof(topic), "sensor_data/Node_%u/gas", node);
    if (!mqttClient.publish(topic, String(gas).c_str(), true)) {
        Serial.println("Gas publish failed");
    }

    snprintf(topic, sizeof(topic), "sensor_data/Node_%u/fire", node);
    if (!mqttClient.publish(topic, fire ? "1" : "0", true)) {
        Serial.println("Fire publish failed");
    }
}

IPAddress getlocalIP() {
  return IPAddress(mesh.getStationIP());
}