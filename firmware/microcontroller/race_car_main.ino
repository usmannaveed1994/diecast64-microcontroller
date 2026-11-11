#include <Arduino.h>
#include <WebServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <math.h>

#include "camaro.h"

// =====================================================
// HARDWARE
// =====================================================

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);

// Panel: a 128x128 sprite is pushed at (0, 0). The glass is mounted 90 degrees
// anticlockwise, which is TFT_eSPI rotation 5 on this display.
#define TFT_SPRITE_WIDTH    128
#define TFT_SPRITE_HEIGHT   128
#define TFT_SCREEN_ROTATION 5
#define TFT_SPRITE_ROTATION 0

// MPU6050
#define SDA_PIN 32
#define SCL_PIN 33
MPU6050 mpu(Wire);

// TTP223 touch sensor
#define TOUCH_PIN 27

// LEDs
#define LED_A_PIN 25   // LEDs 1 and 2
#define LED_B_PIN 26   // LEDs 3 and 4

// =====================================================
// 3D ENGINE
// =====================================================

#define DEG2RAD 0.0174532925f

float currentRoll = 0.0f;
float currentPitch = 0.0f;
float currentYaw = 0.0f;

float targetRoll = 0.0f;
float targetPitch = 0.0f;
float targetYaw = 0.0f;

float px[VERTEX_COUNT];
float py[VERTEX_COUNT];
float pz[VERTEX_COUNT];

uint32_t lastFrame = 0;

// Auto-spin state
float spinOffset = 0.0f;        // extra turn angle from auto-spin (degrees)
float spinSpeed = 0.0f;         // current spin speed, eases toward target
uint32_t lastMotionTime = 0;    // last time the sensor reported movement

// =====================================================
// SETTINGS
// =====================================================

const float CAR_SIZE = 15.0f;
const float CAMERA_DISTANCE = 5.0f;
const float SMOOTHING = 0.18f;
const float VIEW_TURN_OFFSET = 90.0f;   // 90 = side view, try -90 to face the other way

// Projection centre inside the sprite
const float PROJECTION_CENTER_X = 64.0f;
const float PROJECTION_CENTER_Y = 80.0f;

// Auto-spin when the sensor is still
const float MOTION_THRESHOLD = 12.0f;    // gyro deg/s above which we call it "moving"
const uint32_t IDLE_DELAY_MS = 1500;     // stillness needed before auto-spin starts
const float AUTO_SPIN_SPEED = 75.0f;     // degrees per second (negative = spin the other way)
const float SPIN_EASING = 0.15f;         // lower = smoother start/stop

const uint32_t FRAME_INTERVAL_MS = 16;   // approximately 60 FPS

// =====================================================
// DISPLAY
// =====================================================

void initDisplay() {
  tft.init();
  tft.setRotation(TFT_SCREEN_ROTATION);
  tft.fillScreen(TFT_BLACK);

  sprite.setColorDepth(16);

  if (!sprite.createSprite(TFT_SPRITE_WIDTH, TFT_SPRITE_HEIGHT)) {
    Serial.println("Sprite allocation failed!");

    while (true) {
      delay(1000);
    }
  }

  sprite.setRotation(TFT_SPRITE_ROTATION);
  sprite.setTextDatum(TL_DATUM);
}

void drawStatusScreen(const String& title, const String& detail, uint16_t titleColor) {
  sprite.fillSprite(TFT_BLACK);
  sprite.setTextDatum(MC_DATUM);
  sprite.setTextFont(2);
  sprite.setTextSize(1);

  sprite.setTextColor(titleColor, TFT_BLACK);
  sprite.drawString(title, TFT_SPRITE_WIDTH / 2, TFT_SPRITE_HEIGHT / 2 - 10);

  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.drawString(detail.substring(0, 20), TFT_SPRITE_WIDTH / 2, TFT_SPRITE_HEIGHT / 2 + 10);

  sprite.pushSprite(0, 0);
  sprite.setTextDatum(TL_DATUM);
}

void drawProvisioningScreen() {
  sprite.fillSprite(TFT_BLACK);
  sprite.setTextDatum(MC_DATUM);
  sprite.setTextFont(2);
  sprite.setTextSize(1);

  sprite.setTextColor(TFT_CYAN, TFT_BLACK);
  sprite.drawString("WIFI SETUP", TFT_SPRITE_WIDTH / 2, 32);

  sprite.drawFastHLine(20, 46, 88, TFT_MAGENTA);

  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.drawString("JOIN", TFT_SPRITE_WIDTH / 2, 62);

  sprite.setTextColor(TFT_GREEN, TFT_BLACK);
  sprite.drawString("ESP32-Setup-AP", TFT_SPRITE_WIDTH / 2, 80);

  sprite.setTextColor(TFT_YELLOW, TFT_BLACK);
  sprite.drawString("OPEN WEB APP", TFT_SPRITE_WIDTH / 2, 106);

  sprite.pushSprite(0, 0);
  sprite.setTextDatum(TL_DATUM);
}

// =====================================================
// BOOT ANIMATION
// =====================================================

void bootAnimation() {
  sprite.fillSprite(TFT_BLACK);

  sprite.setTextDatum(MC_DATUM);
  sprite.setTextColor(TFT_CYAN, TFT_BLACK);
  sprite.setTextSize(2);
  sprite.drawString("DIECAST 64", 64, 12);

  sprite.drawFastHLine(18, 47, 92, TFT_MAGENTA);

  sprite.setTextSize(1);
  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.drawString("CYBER SYSTEM", 64, 62);

  const int barX = 15;
  const int barY = 79;
  const int barW = 98;
  const int barH = 10;

  sprite.drawRect(barX, barY, barW, barH, TFT_CYAN);

  for (int progress = 0; progress <= 100; progress += 4) {
    int fillW = (barW - 4) * progress / 100;

    sprite.fillRect(
      barX + 2, barY + 2,
      barW - 4, barH - 4,
      TFT_BLACK
    );

    if (fillW > 0) {
      sprite.fillRect(
        barX + 2, barY + 2,
        fillW, barH - 4,
        TFT_MAGENTA
      );
    }

    sprite.setTextColor(TFT_GREEN, TFT_BLACK);
    sprite.fillRect(40, 96, 48, 10, TFT_BLACK);
    sprite.drawCentreString(
      String(progress) + "%",
      64, 96, 1
    );

    sprite.pushSprite(0, 0);
    delay(35);
  }

  delay(400);

  sprite.fillSprite(TFT_BLACK);

  sprite.setTextDatum(MC_DATUM);

  sprite.setTextColor(TFT_GREEN, TFT_BLACK);
  sprite.drawString("SYSTEM ONLINE", 64, 51);

  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.drawString("ESP32 CORE", 64, 69);

  sprite.setTextColor(TFT_YELLOW, TFT_BLACK);
  sprite.drawString("BOOT SUCCESS", 64, 88);

  sprite.pushSprite(0, 0);
  delay(1500);

  sprite.setTextDatum(TL_DATUM);
}

// =====================================================
// FLOATING TEXT
// =====================================================

const uint8_t TEXT_BAND_HEIGHT = 24;
const uint8_t TEXT_FONT_HEIGHT = 16;       // TFT_eSPI font 2
const size_t FLOATING_TEXT_MAX_LENGTH = 64;
const float TEXT_SCROLL_SPEED = 34.0f;     // pixels per second

String floatingText;
int floatingTextWidth = 0;
float floatingTextScroll = 0.0f;

void setFloatingText(const String& text) {
  floatingText = text.substring(0, FLOATING_TEXT_MAX_LENGTH);
  floatingTextScroll = 0.0f;

  sprite.setTextFont(2);
  sprite.setTextSize(1);
  floatingTextWidth = floatingText.length() > 0 ? sprite.textWidth(floatingText) : 0;
}

void updateFloatingText(float dt) {
  if (floatingText.length() == 0) {
    floatingTextScroll = 0.0f;
    return;
  }

  // Always marquee horizontally, regardless of whether the text fits.
  floatingTextScroll += TEXT_SCROLL_SPEED * dt;

  if (floatingTextScroll > floatingTextWidth + TFT_SPRITE_WIDTH) {
    floatingTextScroll = 0.0f;
  }
}

int floatingTextX() {
  // Text enters from the right edge and exits past the left edge.
  return TFT_SPRITE_WIDTH - (int)floatingTextScroll;
}

void drawFloatingText() {
  if (floatingText.length() == 0) return;

  const int bandTop = TFT_SPRITE_HEIGHT - TEXT_BAND_HEIGHT;

  // Clear the band so the marquee does not leave a trail over the car.
  sprite.fillRect(0, bandTop, TFT_SPRITE_WIDTH, TEXT_BAND_HEIGHT, TFT_BLACK);
  sprite.drawFastHLine(0, bandTop, TFT_SPRITE_WIDTH, TFT_MAGENTA);

  sprite.setTextDatum(TL_DATUM);
  sprite.setTextFont(2);
  sprite.setTextSize(1);
  sprite.setTextColor(TFT_CYAN);

  // Horizontally scrolling only: the vertical position is fixed.
  const int y = bandTop + (TEXT_BAND_HEIGHT - TEXT_FONT_HEIGHT) / 2;

  sprite.drawString(floatingText, floatingTextX(), y);
}

// =====================================================
// TOUCH ANIMATION
// =====================================================

void drawTouchAnimation() {
  const uint32_t now = millis();

  const uint16_t cycle = now % 1200;
  const float phase = cycle / 1200.0f;

  sprite.fillSprite(TFT_BLACK);

  sprite.drawRect(2, 2, 124, 124, TFT_MAGENTA);
  sprite.drawRect(5, 5, 118, 118, TFT_BLUE);

  for (int ring = 0; ring < 3; ring++) {
    float ringPhase = fmodf(
      phase + ring * 0.3333f,
      1.0f
    );

    int radius = 9 + (int)(ringPhase * 43.0f);

    uint16_t ringColor =
      (ring == 1) ? TFT_MAGENTA : TFT_CYAN;

    sprite.drawCircle(64, 57, radius, ringColor);
  }

  int pulseRadius = 5 + (int)(
    3.0f * (0.5f + 0.5f * sinf(now * 0.012f))
  );

  sprite.fillCircle(64, 57, pulseRadius, TFT_GREEN);

  sprite.setTextDatum(MC_DATUM);

  sprite.setTextColor(TFT_CYAN, TFT_BLACK);
  sprite.drawString("TOUCH DETECTED", 64, 15, 1);

  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.drawString("CYBER INTERFACE", 64, 103, 1);

  sprite.setTextColor(TFT_GREEN, TFT_BLACK);
  sprite.drawString("HOLD TO ACTIVATE", 64, 115, 1);

  sprite.pushSprite(0, 0);

  sprite.setTextDatum(TL_DATUM);
}

// =====================================================
// 3D CAR RENDERER
// =====================================================

void drawCar(float roll, float pitch, float yaw) {
  float r = roll * DEG2RAD;
  float p = (pitch + VIEW_TURN_OFFSET + spinOffset) * DEG2RAD;
  float y = yaw * DEG2RAD;

  float sr = sinf(r), cr = cosf(r);
  float sp = sinf(p), cp = cosf(p);
  float sy = sinf(y), cy = cosf(y);

  for (uint16_t i = 0; i < VERTEX_COUNT; i++) {
    float x = carVertices[i][0];
    float yy = carVertices[i][1];
    float z = carVertices[i][2];

    // X rotation
    float x1 = x;
    float y1 = yy * cr - z * sr;
    float z1 = yy * sr + z * cr;

    // Y rotation
    float x2 = x1 * cp + z1 * sp;
    float y2 = y1;
    float z2 = -x1 * sp + z1 * cp;

    // Z rotation
    float x3 = x2 * cy - y2 * sy;
    float y3 = x2 * sy + y2 * cy;
    float z3 = z2;

    float denominator = CAMERA_DISTANCE - z3;

    if (denominator < 0.5f) {
      denominator = 0.5f;
    }

    float perspective = CAMERA_DISTANCE / denominator;

    px[i] = PROJECTION_CENTER_X + x3 * CAR_SIZE * perspective;
    py[i] = PROJECTION_CENTER_Y - y3 * CAR_SIZE * perspective;
    pz[i] = z3;
  }

  for (uint16_t i = 0; i < EDGE_COUNT; i++) {
    uint16_t a = carEdges[i][0];
    uint16_t b = carEdges[i][1];

    uint16_t edgeColor =
      (pz[a] + pz[b] > 0.0f)
      ? TFT_CYAN
      : TFT_BLUE;

    sprite.drawLine(
      (int)px[a], (int)py[a],
      (int)px[b], (int)py[b],
      edgeColor
    );
  }
}

// =====================================================
// NORMAL CAR SCREEN
// =====================================================

void drawCarScreen() {
  sprite.fillSprite(TFT_BLACK);

  sprite.setTextDatum(TL_DATUM);
  sprite.setTextColor(TFT_CYAN, TFT_BLACK);
  sprite.setTextFont(2);
  sprite.setTextSize(1);
  sprite.setCursor(4, 4);
  sprite.print("DIECAST64 GARAGE");

  drawCar(
    currentRoll,
    currentPitch,
    currentYaw
  );

  drawFloatingText();

  sprite.pushSprite(0, 0);
}

// =====================================================
// REMOTE CONTROL (MQTT)
// =====================================================

const char* MQTT_HOST = "broker.hivemq.com";
const uint16_t MQTT_PORT = 1883;
const char* TOPIC_PREFIX = "usman-esp32-9988";
const uint32_t MQTT_RETRY_MS = 5000;

WiFiClient netClient;
PubSubClient mqtt(netClient);

bool ledAOn = false;
bool ledBOn = false;

String topicLedASet;
String topicLedAState;
String topicLedBSet;
String topicLedBState;
String topicTextSet;
String topicTextState;

void initTopics() {
  const String ledRoot = String(TOPIC_PREFIX) + "/led";
  const String textRoot = String(TOPIC_PREFIX) + "/text";

  topicLedASet = ledRoot + "/a/set";
  topicLedAState = ledRoot + "/a/state";
  topicLedBSet = ledRoot + "/b/set";
  topicLedBState = ledRoot + "/b/state";

  topicTextSet = textRoot + "/set";
  topicTextState = textRoot + "/state";
}

void publishLedStates() {
  mqtt.publish(topicLedAState.c_str(), ledAOn ? "ON" : "OFF", true);
  mqtt.publish(topicLedBState.c_str(), ledBOn ? "ON" : "OFF", true);
}

void publishFloatingTextState() {
  mqtt.publish(topicTextState.c_str(), floatingText.c_str(), true);
}

void applyFloatingText(const String& text) {
  setFloatingText(text);
  publishFloatingTextState();

  Serial.print("TFT text: ");
  Serial.println(floatingText);
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  // Text must keep its original casing and spacing, so only trim the edges.
  String message;
  message.reserve(length);

  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  message.trim();

  const String topicName(topic);

  if (topicName == topicTextSet) {
    applyFloatingText(message);
    return;
  }

  if (topicName == topicLedASet) {
    ledAOn = message.equalsIgnoreCase("ON");
    digitalWrite(LED_A_PIN, ledAOn ? HIGH : LOW);
    publishLedStates();
    return;
  }

  if (topicName == topicLedBSet) {
    ledBOn = message.equalsIgnoreCase("ON");
    digitalWrite(LED_B_PIN, ledBOn ? HIGH : LOW);
    publishLedStates();
    return;
  }
}

// =====================================================
// WIFI PROVISIONING
// =====================================================

WebServer server(80);
Preferences preferences;

bool provisioningMode = false;

const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
const uint32_t WIFI_RETRY_MS = 10000;

bool loadWifiCredentials(String& ssid, String& password) {
  preferences.begin("wifi", true);
  ssid = preferences.getString("ssid", "");
  password = preferences.getString("pass", "");
  preferences.end();

  return ssid.length() > 0;
}

bool connectToStoredWifi() {
  String ssid;
  String password;

  if (!loadWifiCredentials(ssid, password)) {
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());

  drawStatusScreen("CONNECTING", ssid, TFT_CYAN);

  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
  }

  return WiFi.status() == WL_CONNECTED;
}

void handleWifiSave() {
  if (server.hasArg("plain") == false) {
    server.send(400, "text/plain", "Bad Request");
    return;
  }

  // Expected body: {"ssid":"...","password":"..."}
  String body = server.arg("plain");

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
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32-Setup-AP", "12345678");

  server.on("/api/wifi", HTTP_POST, handleWifiSave);
  // Handle CORS preflight requests from the Next.js app
  server.on("/api/wifi", HTTP_OPTIONS, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "POST");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    server.send(204);
  });

  server.begin();

  provisioningMode = true;
  drawProvisioningScreen();

  Serial.println("Started provisioning AP: ESP32-Setup-AP");
}

// =====================================================
// SENSORS
// =====================================================

bool sensorReady = false;

void initSensors() {
  Wire.begin(SDA_PIN, SCL_PIN);

  const byte status = mpu.begin();

  if (status != 0) {
    Serial.println("MPU6050 connection failed!");
    drawStatusScreen("MPU6050 ERROR", "CHECK WIRING", TFT_RED);
    return;
  }

  drawStatusScreen("CALIBRATING", "KEEP LEVEL", TFT_CYAN);
  delay(700);

  mpu.calcOffsets();
  mpu.update();

  currentRoll = targetRoll = mpu.getAngleX();
  currentPitch = targetPitch = mpu.getAngleY();
  currentYaw = targetYaw = mpu.getAngleZ() + 180.0f;  // face the car towards you

  sensorReady = true;
}

void updateSensors() {
  if (!sensorReady) return;

  mpu.update();

  targetRoll = mpu.getAngleX();
  targetPitch = mpu.getAngleY();
  targetYaw = mpu.getAngleZ();

  const float gyroMagnitude =
    fabsf(mpu.getGyroX()) + fabsf(mpu.getGyroY()) + fabsf(mpu.getGyroZ());

  if (gyroMagnitude > MOTION_THRESHOLD) {
    lastMotionTime = millis();
  }
}

void updateOrientation(float dt) {
  // Smooth movement from the sensor
  currentRoll += (targetRoll - currentRoll) * SMOOTHING;
  currentPitch += (targetPitch - currentPitch) * SMOOTHING;
  currentYaw += (targetYaw - currentYaw) * SMOOTHING;

  // Auto-spin when still, ease back to zero when moved
  const bool idle = (millis() - lastMotionTime) > IDLE_DELAY_MS;
  const float targetSpin = idle ? AUTO_SPIN_SPEED : 0.0f;

  spinSpeed += (targetSpin - spinSpeed) * SPIN_EASING;
  spinOffset += spinSpeed * dt;

  if (spinOffset >= 360.0f) spinOffset -= 360.0f;
  if (spinOffset < 0.0f) spinOffset += 360.0f;
}

void renderFrame() {
  if (digitalRead(TOUCH_PIN) == HIGH) {
    drawTouchAnimation();
  } else {
    drawCarScreen();
  }
}

// =====================================================
// CONNECTION MANAGEMENT
// =====================================================

void initMqtt() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(512);   // room for full-length floating-text payloads
}

void remoteLoop() {
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastWifiRetry = 0;
    if (millis() - lastWifiRetry > WIFI_RETRY_MS) {
      lastWifiRetry = millis();
      WiFi.reconnect();
    }
    return;
  }

  if (!mqtt.connected()) {
    static uint32_t lastMqttAttempt = 0;
    if (millis() - lastMqttAttempt < MQTT_RETRY_MS) return;
    lastMqttAttempt = millis();

    const String clientId = "esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);

    if (!mqtt.connect(clientId.c_str())) {
      Serial.print("MQTT failed, state ");
      Serial.println(mqtt.state());
      return;
    }

    Serial.println("MQTT connected");

    mqtt.subscribe(topicLedASet.c_str());
    mqtt.subscribe(topicLedBSet.c_str());
    mqtt.subscribe(topicTextSet.c_str());

    publishLedStates();
    publishFloatingTextState();
  }

  mqtt.loop();
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  pinMode(TOUCH_PIN, INPUT);

  // LEDs off at boot
  pinMode(LED_A_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  digitalWrite(LED_A_PIN, LOW);
  digitalWrite(LED_B_PIN, LOW);

  initDisplay();
  bootAnimation();

  initTopics();
  initSensors();

  if (!connectToStoredWifi()) {
    startProvisioningAP();
    return;
  }

  initMqtt();

  lastFrame = millis();
  lastMotionTime = millis();

  Serial.println("DIECAST 64 SYSTEM ONLINE");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  if (provisioningMode) {
    server.handleClient();
    delay(5);
    return;
  }

  remoteLoop();
  updateSensors();

  const uint32_t now = millis();

  if (now - lastFrame < FRAME_INTERVAL_MS) {
    return;
  }

  float dt = (now - lastFrame) / 1000.0f;
  if (dt > 0.1f) dt = 0.1f;   // guard against a long frame
  lastFrame = now;

  updateOrientation(dt);
  updateFloatingText(dt);
  renderFrame();
}
