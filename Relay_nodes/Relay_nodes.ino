#include "painlessMesh.h"
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>

#define MESH_PREFIX "Fire_Gas_Detector"
#define MESH_PASSWORD "saveslifefromfire"
#define MESH_PORT 5555

const short room = 2; //Assign room number for the room you want the sensor to be installed

const short firepin = D5;  // Fire sensor pin
const short gaspin = D2;   // Gas sensor pin
const short outpin = D6;   // Alert output pin

bool manualReset = false;    // Manual reset flag


Scheduler myScheduler; // to schedule task
painlessMesh  mesh;

// Function prototypes
void sendMessage(); 
void readSensor();
void checkConnections(); 

Task taskSendData( TASK_SECOND * 1 , TASK_FOREVER, &sendMessage );
Task taskReadSensors(TASK_MILLISECOND * 100, TASK_FOREVER, &readSensor);
Task taskCheckConnections(TASK_SECOND * 5, TASK_FOREVER, &checkConnections);

void sendMessage() {
  // Read sensor data
    short firesensor = digitalRead(firepin);
    short gassensor = digitalRead(gaspin);

    // Create JSON data
    StaticJsonDocument<100> jsonDat;
    jsonDat["room"] = room;
    jsonDat["firesensor"] = firesensor;
    jsonDat["gassensor"] = gassensor;

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
    taskSendData.setInterval( random( TASK_SECOND * 1, TASK_SECOND * 5 ));
}

  void readSensor() {
    if (!manualReset) {
      if (digitalRead(firepin) == HIGH || digitalRead(gaspin) == HIGH) {
        digitalWrite(outpin, HIGH);
      } 
      else {
        digitalWrite(outpin, LOW);
      }
    }
  }

  void checkConnections() {
    auto nodes = mesh.getNodeList();  // Get a list of connected nodes
    if (nodes.size() == 0) {
      Serial.println("No active nodes found in the mesh network.");
    } else {
      Serial.printf("Active nodes: %u\n", nodes.size());
    }
  }

// Needed for painless library
void receivedCallback( uint32_t from, String &msg ) {
  StaticJsonDocument<50> doc;

  DeserializationError error = deserializeJson(doc, msg);

  if (!error){
    short room = doc["room"];
    bool fireSensor = doc["firesensor"];
    bool gasSensor = doc["gassensor"];

    Serial.printf("Room %d, Fire: %d, Gas: %d\n", room, fireSensor, gasSensor);
  }
  else{
    if (msg.startsWith("RESET")) {
    manualReset = true;
    digitalWrite(outpin, LOW);
    Serial.println("Manual reset activated.");
    }
    else if(msg.startsWith("CLEAR_RESET")){
      manualReset = false;
      Serial.println("Manual reset cleared.");
    }
    else{
      Serial.println("Received no json data.");
    }
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

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP_STA);    // Required for painlessMesh
  pinMode(firepin, INPUT);
  pinMode(gaspin, INPUT);
  digitalWrite(outpin, LOW);
  pinMode(outpin, OUTPUT);


  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages

  //Initialize mesh network
  mesh.init( MESH_PREFIX, MESH_PASSWORD, &myScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  myScheduler.addTask(taskSendData);
  myScheduler.addTask(taskReadSensors);
  myScheduler.addTask(taskCheckConnections);

  taskSendData.enable();
  taskReadSensors.enable();
  taskCheckConnections.enable();
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
  myScheduler.execute();
}
