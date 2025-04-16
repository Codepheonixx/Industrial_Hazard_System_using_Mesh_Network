#include "painlessMesh.h"
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>

#define MESH_PREFIX "Fire_Gas_Detector"
#define MESH_PASSWORD "saveslifefromfire"
#define MESH_PORT 5555

const uint8_t room = 2; //Assign room number for the room you want the sensor to be installed

const short firepin = D5;  // Fire sensor pin
const short gaspin = A0;   // Gas sensor pin
const short buzzer = D6;   // Alert output pin
uint16_t gas_threshold = 300;   // Set the gas threshold

bool firesensor;
uint16_t gassensor;

bool buzzerActive = false;
unsigned long lastBlinkTime = 0;
const unsigned long BUZZER_BLINK_INTERVAL = 500; // blink every 500ms
bool buzzerState = false; // tracks ON/OFF state of buzzer for blinking

Scheduler myScheduler; // to schedule task
painlessMesh  mesh;

// Function prototypes
void sendMessage(); 
void readSensor();
void receivedCallback(uint32_t from, String &msg); 

Task taskSendData( TASK_SECOND * 1 , TASK_FOREVER, &sendMessage );
Task taskReadSensors(TASK_MILLISECOND * 100, TASK_FOREVER, &readSensor);

void sendMessage() {
    // Create JSON data
    StaticJsonDocument<100> jsonDat;
    jsonDat["node"] = room;
    jsonDat["fire"] = firesensor;
    jsonDat["gas"] = gassensor;

    String jsonString;
    serializeJson(jsonDat, jsonString);

    // Send data to the every node
    if (mesh.sendBroadcast(jsonString)) {
    Serial.println("Broadcast successful");
    } 
    else {
    Serial.println("Broadcast failed");
    }

    Serial.println("Sensor data " + jsonString);
    taskSendData.setInterval( random( TASK_MILLISECOND * 500, TASK_MILLISECOND * 1500 ));
}

void readSensor() {
  firesensor = digitalRead(firepin);
  gassensor = analogRead(gaspin);

  // Alarm Trigger
  if ((firesensor || gassensor > gas_threshold) && !buzzerActive) {
    buzzerActive = true;
    Serial.printf("ALARM: Fire/Gas detected in Room %d!\n", room);
  }
}


// Needed for painless library
void receivedCallback(uint32_t from, String &msg) {
  StaticJsonDocument<256> doc;

  // Check for raw "reset" command first
  if (msg == "reset") {
    Serial.println("Reset message received from gateway");
    buzzerActive = false;
    return;
  }

  // Then try to parse JSON if it's not a raw command
  DeserializationError error = deserializeJson(doc, msg);
  if (error) {
    Serial.printf("JSON error: %s\n", error.c_str());
    return;
  }

  // Handle valid JSON structure
  short room = doc["node"];
  bool fireSensor = doc["fire"];
  bool gasSensor = doc["gas"];

  Serial.printf("Room %d, Fire: %d, Gas: %d\n", room, fireSensor, gasSensor);
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

void setup() {
  Serial.begin(115200);
  pinMode(firepin, INPUT);
  pinMode(gaspin, INPUT);
  pinMode(buzzer, OUTPUT);

  digitalWrite(firepin,LOW);
  digitalWrite(buzzer,LOW);

  // Initialize Wi-Fi mode and channel FIRST
  WiFi.mode(WIFI_AP_STA);       // Dual mode (AP + Station)

  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages
  mesh.init( MESH_PREFIX, MESH_PASSWORD, &myScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  
  // This node and all other nodes should ideally know the mesh contains a root, so call this on all nodes
  mesh.setContainsRoot(true);

  myScheduler.addTask(taskSendData);
  myScheduler.addTask(taskReadSensors);

  taskSendData.enable();
  taskReadSensors.enable();
}

void loop() {
  
  // it will run the user scheduler as well
  mesh.update();
  myScheduler.execute();

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
