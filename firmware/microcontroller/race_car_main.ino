#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <PubSubClient.h>

Preferences preferences;
WebServer server(80);
WiFiClient plainClient;
PubSubClient mqtt(plainClient);

const char* MQTT_HOST = "broker.hivemq.com";
const int   MQTT_PORT = 1883;
const char* TOPIC_PREFIX = "usman-esp32-9988/led";

#define LED_A_PIN 25
#define LED_B_PIN 26

String topicSetA, topicSetB, topicStateA, topicStateB;
bool ledAOn = false;
bool ledBOn = false;

void publishStates() {
  mqtt.publish(topicStateA.c_str(), ledAOn ? "ON" : "OFF", true);
  mqtt.publish(topicStateB.c_str(), ledBOn ? "ON" : "OFF", true);
}

void onMqttMessage(char* topic, byte* payload, unsigned int len) {
  String msg = "";
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  msg.trim();
  msg.toUpperCase();
  
  bool on = (msg == "ON");
  String t = topic;

  if (t == topicSetA) {
    ledAOn = on;
    digitalWrite(LED_A_PIN, on);
  } else if (t == topicSetB) {
    ledBOn = on;
    digitalWrite(LED_B_PIN, on);
  }
  publishStates();
}

void handleWifiSave() {
  if (server.hasArg("plain") == false) {
    server.send(400, "text/plain", "Bad Request");
    return;
  }
  
  // Simple JSON parsing for SSID and Password
  String body = server.arg("plain");
  // Expected: {"ssid":"...","password":"..."}
  int sIdx = body.indexOf("\"ssid\":\"") + 8;
  int sEnd = body.indexOf("\"", sIdx);
  String ssid = body.substring(sIdx, sEnd);

  int pIdx = body.indexOf("\"password\":\"") + 12;
  int pEnd = body.indexOf("\"", pIdx);
  String pass = body.substring(pIdx, pEnd);

  preferences.begin("wifi", false);
  preferences.putString("ssid", ssid);
  preferences.putString("pass", pass);
  preferences.end();

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"status\":\"saved\"}");
  delay(1000);
  ESP.restart();
}

void startProvisioningAP() {
  WiFi.softAP("ESP32-Setup-AP", "12345678");
  Serial.println("Started Provisioning AP: ESP32-Setup-AP");

  server.on("/api/wifi", HTTP_POST, handleWifiSave);
  // Handle CORS preflight requests from Next.js
  server.on("/api/wifi", HTTP_OPTIONS, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "POST");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    server.send(204);
  });
  
  server.begin();
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_A_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);

  topicSetA   = String(TOPIC_PREFIX) + "/a/set";
  topicSetB   = String(TOPIC_PREFIX) + "/b/set";
  topicStateA = String(TOPIC_PREFIX) + "/a/state";
  topicStateB = String(TOPIC_PREFIX) + "/b/state";

  preferences.begin("wifi", true);
  String ssid = preferences.getString("ssid", "");
  String pass = preferences.getString("pass", "");
  preferences.end();

  if (ssid == "") {
    startProvisioningAP();
  } else {
    WiFi.begin(ssid.c_str(), pass.c_str());
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() != WL_CONNECTED) {
      startProvisioningAP();
    } else {
      mqtt.setServer(MQTT_HOST, MQTT_PORT);
      mqtt.setCallback(onMqttMessage);
    }
  }
}

void loop() {
  if (WiFi.getMode() == WIFI_AP) {
    server.handleClient();
  } else {
    if (!mqtt.connected()) {
      String clientId = "esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
      if (mqtt.connect(clientId.c_str())) {
        mqtt.subscribe(topicSetA.c_str());
        mqtt.subscribe(topicSetB.c_str());
        publishStates();
      } else {
        delay(5000);
      }
    }
    mqtt.loop();
  }
}