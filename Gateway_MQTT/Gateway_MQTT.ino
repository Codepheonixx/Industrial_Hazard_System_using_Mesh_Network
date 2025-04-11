#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>

#define   STATION_SSID     "TEST_NETWORK_1"
#define   STATION_PASSWORD "12345678"

#define HOSTNAME "FireAlarm_Gateway"

#define RX_PIN D1  // GPIO5
#define TX_PIN D2  // GPIO4

const uint8_t buzzer = D6;   // Buzzer output pin
short gas_threshold = 600;   // Set the gas threshold

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

void mqttCallback(char* topic, byte* payload, unsigned int length);

SoftwareSerial softSerial(RX_PIN, TX_PIN);  // RX, TX
WiFiClient wifiClient;
PubSubClient mqttClient(mqttBroker, 1884, mqttCallback, wifiClient); //Set MQTT

String input = "";

void setup() {

  Serial.begin(115200);
  softSerial.begin(9600);          // SoftwareSerial (match baud rate on other ESP)

  pinMode(buzzer, OUTPUT);

  digitalWrite(buzzer,LOW);

  WiFi.mode(WIFI_STA);            // Station mode
  WiFi.hostname(HOSTNAME);
  WiFi.begin(STATION_SSID, STATION_PASSWORD);
  
  Serial.print("Connecting");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Hostname: ");
  Serial.println(WiFi.hostname());
}

void loop(){

  mqttClient.loop();  // Process MQTT messages

  mqttPub();
  
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

void mqttPub() {
  while (softSerial.available()) {
    char c = softSerial.read();
    if (c == '\n') {
      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, input);
      if (!error) {
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
      } else {
        Serial.print("JSON Error: ");
        Serial.println(error.c_str());
      }
      input = "";
    } else {
      input += c;
    }
  }
}

bool reconnect() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    WiFi.begin(STATION_SSID, STATION_PASSWORD);
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
    softSerial.println("reset");
  }
}

IPAddress getlocalIP() {
  return IPAddress(WiFi.localIP());
}




