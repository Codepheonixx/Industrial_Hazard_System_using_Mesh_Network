#include "painlessMesh.h"
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

#define RX_PIN D1  // GPIO5
#define TX_PIN D2  // GPIO4

#define MESH_PREFIX "Fire_Gas_Detector"
#define MESH_PASSWORD "saveslifefromfire"
#define MESH_PORT 5555

const uint8_t firepin = D5;  // Fire sensor pin
const uint8_t gaspin = A0;   // Gas sensor pin

void receivedCallback(uint32_t from, String &msg);

painlessMesh mesh;
SoftwareSerial softSerial(RX_PIN, TX_PIN);  // RX, TX

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
  softSerial.begin(9600);          // SoftwareSerial (match baud rate on other ESP)

  pinMode(firepin, INPUT);
  pinMode(gaspin, INPUT);

  digitalWrite(firepin,LOW);

  // Initialize mesh network
  mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION | SYNC);
  mesh.init( MESH_PREFIX, MESH_PASSWORD, MESH_PORT, WIFI_AP_STA );
  mesh.onReceive(&receivedCallback);
  mesh.setRoot(true);
  mesh.setContainsRoot(true);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

}

void loop(){
  mesh.update();      // Handle mesh network

  if (softSerial.available()) {
    String msg = softSerial.readStringUntil('\n');    // Read until newline
    msg.trim();                                       // Remove any whitespace/newline characters

    if (msg == "reset") {
      Serial.println("Reset command received. Broadcasting...");
      if (mesh.sendBroadcast("reset")) {
        Serial.println("Broadcasted successfully.");
      } else {
        Serial.println("Broadcast failed.");
      }
    }
  }
  
  // Reads sensor every 1s
  static unsigned long lastRead = 0;
  if (millis() - lastRead >= 1000) {
    lastRead = millis();
    readSensor();
  }
}

// reads sensor
void readSensor() {

  short gas = analogRead(gaspin);
  bool fire = digitalRead(firepin);

  StaticJsonDocument<256> doc;
  doc["node"] = 0;
  doc["fire"] = fire;
  doc["gas"]  = gas;

  String output;
  serializeJson(doc, output);
  softSerial.println(output); // Newline as delimiter

  Serial.println("Sent: " + output);

}

// Gets callbacks from various nodes
void receivedCallback(uint32_t from, String &msg) {

    //Send data to ESP using serial
    softSerial.println(msg);
    Serial.printf("[Mesh] Msg: %s\nSent to ESP-MQTT\n", msg.c_str());

}

