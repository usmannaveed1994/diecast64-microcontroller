#include <Arduino.h>
#include <WebServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <math.h>
#include <LittleFS.h>
#include <driver/i2s.h>

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
//  AUDIO - MAX98357A
// =====================================================

#define NOTIFICATION_AUDIO_BCLK 21
#define NOTIFICATION_AUDIO_LRC  22
#define NOTIFICATION_AUDIO_DOUT 2

#define NOTIFICATION_AUDIO_SAMPLE_RATE 44100
#define NOTIFICATION_AUDIO_FILE "/notification-click.wav"
#define LIGHTS_AUDIO_FILE "/lights.wav"

TaskHandle_t notificationAudioTaskHandle = nullptr;

bool notificationAudioReady = false;
volatile bool notificationAudioPlaying = false;

// =====================================================
// 3D ENGINE
// =====================================================

#define DEG2RAD 0.0174532925f

float currentRoll = 0.0f;
float currentPitch = 0.0f;
float currentYaw = 0.0f;

float targetRoll = 0.0f;
float targetPitch = 0.0f;

// Previous raw targets, used to notice movement even when the gyro rate is low.
float previousRoll = 0.0f;
float previousPitch = 0.0f;

float px[VERTEX_COUNT];
float py[VERTEX_COUNT];
float pz[VERTEX_COUNT];

uint32_t lastFrame = 0;
float measuredFps = 0.0f;

// Latest sensor sample. Refreshed on every loop pass (not only once per rendered
// frame) so a quick flick of the board is never missed.
float gyroX = 0.0f;
float gyroY = 0.0f;
float gyroZ = 0.0f;
float gyroMagnitude = 0.0f;     // length of the gyro vector, degrees/second
float accX = 0.0f;
float accY = 0.0f;
float accZ = 0.0f;

// Gyro driven yaw. It is integrated only while the sensor is moving and unwinds
// while it is still, so the gyro bias can never keep the car turning for ever.
float yawAngle = 0.0f;          // yaw target, degrees
float yawRate = 0.0f;           // low passed gyro Z, degrees/second

// Motion detection
bool sensorMoving = false;
uint32_t lastMotionTime = 0;    // last time the sensor reported movement

// Auto-spin state
float spinOffset = 0.0f;        // extra turn angle from auto-spin (degrees)
float spinBlend = 0.0f;         // 0 = gyro drives the car, 1 = auto-spin
float spinSpeed = 0.0f;         // spin speed applied this frame, degrees/second

// =====================================================
// SETTINGS
// =====================================================

const float CAR_SIZE = 15.0f;
const float CAMERA_DISTANCE = 5.0f;
const float SMOOTHING = 0.18f;
const float VIEW_TURN_OFFSET = 90.0f;   // 90 = side view, try -90 to face the other way
const float YAW_VIEW_OFFSET = 0.0f;     // 180 = look at the other end of the car

// Projection centre inside the sprite
const float PROJECTION_CENTER_X = 64.0f;
const float PROJECTION_CENTER_Y = 80.0f;

// Motion detection. The hysteresis (enter > exit) stops the car from flickering
// between "follow the gyro" and "auto-spin" while the readings hover on the limit.
const float MOTION_ENTER_THRESHOLD = 8.0f;   // gyro vector length, degrees/second
const float MOTION_EXIT_THRESHOLD = 3.0f;    // must drop below this to count as still
const float MOTION_ANGLE_THRESHOLD = 1.5f;   // deg of tilt change per frame (fast moves only)
const uint32_t IDLE_DELAY_MS = 1500;         // stillness needed before auto-spin starts

// Auto-spin when the sensor is still
const float AUTO_SPIN_SPEED = 75.0f;     // degrees per second (negative = spin the other way)
const float SPIN_STOP_TIME = 0.12f;      // seconds for the spin to get out of the way
const float SPIN_START_TIME = 0.35f;     // seconds for the spin to ease back in

// Yaw (gyro Z) handling
const float YAW_RATE_SMOOTHING_TIME = 0.05f;  // seconds, low pass on the gyro rate
const float YAW_GYRO_DEADBAND = 0.8f;         // degrees/second ignored as noise
const float YAW_RETURN_TIME = 1.2f;           // seconds for the yaw to unwind when still

const uint32_t FRAME_INTERVAL_MS = 16;   // approximately 60 FPS

// Serial debug: set to 0 to silence the [dbg] output.
#define DEBUG_SERIAL 1

const uint32_t DEBUG_INTERVAL_MS = 250;      // how often the state line is printed
const uint32_t DEBUG_SENSOR_WARN_MS = 2000;  // how often to nag about a dead sensor

// =====================================================
// SERIAL DEBUG
// =====================================================

#if DEBUG_SERIAL
uint32_t lastDebugPrint = 0;
uint32_t lastSensorWarn = 0;
#endif

uint32_t stillTimeMs() {
  return millis() - lastMotionTime;
}

void debugPrintConfig() {
#if DEBUG_SERIAL
  Serial.println();
  Serial.println("----- DIECAST 64 DEBUG -----");
  Serial.printf(
    "motion: enter=%.1f deg/s exit=%.1f deg/s tilt=%.1f deg/frame idle=%lu ms\n",
    (double)MOTION_ENTER_THRESHOLD,
    (double)MOTION_EXIT_THRESHOLD,
    (double)MOTION_ANGLE_THRESHOLD,
    (unsigned long)IDLE_DELAY_MS
  );
  Serial.printf(
    "spin: %.1f deg/s, stop %.2f s, start %.2f s\n",
    (double)AUTO_SPIN_SPEED,
    (double)SPIN_STOP_TIME,
    (double)SPIN_START_TIME
  );
  Serial.printf(
    "yaw: deadband %.2f deg/s, return %.2f s\n",
    (double)YAW_GYRO_DEADBAND,
    (double)YAW_RETURN_TIME
  );
  Serial.println("----------------------------");
#endif
}

// One compact line every DEBUG_INTERVAL_MS. "MOVE" means the gyro is driving
// the car, "IDLE" means it is auto-spinning. Paste this back when debugging.
void debugPrintState() {
#if DEBUG_SERIAL
  const uint32_t now = millis();
  if (now - lastDebugPrint < DEBUG_INTERVAL_MS) return;
  lastDebugPrint = now;

  Serial.printf(
    "[dbg] t=%lu g=(%6.2f,%6.2f,%6.2f) |g|=%6.2f a=(%5.2f,%5.2f,%5.2f) "
    "raw=(%7.2f,%7.2f) rpy=(%7.2f,%7.2f,%7.2f) %s still=%5lums "
    "spin=%.2f spd=%6.1f off=%6.1f fps=%4.1f\n",
    (unsigned long)now,
    (double)gyroX, (double)gyroY, (double)gyroZ, (double)gyroMagnitude,
    (double)accX, (double)accY, (double)accZ,
    (double)targetRoll, (double)targetPitch,
    (double)currentRoll, (double)currentPitch, (double)currentYaw,
    sensorMoving ? "MOVE" : "IDLE",
    (unsigned long)stillTimeMs(),
    (double)spinBlend, (double)spinSpeed, (double)spinOffset,
    (double)measuredFps
  );
#endif
}

void debugLogMotion(bool moving) {
#if DEBUG_SERIAL
  Serial.printf(
    "[motion] %s |g|=%.2f deg/s still=%lums yaw=%.2f\n",
    moving ? "START gyro follow" : "STOP  auto-spin resumes",
    (double)gyroMagnitude,
    (unsigned long)stillTimeMs(),
    (double)yawAngle
  );
#endif
}

void debugWarnSensor() {
#if DEBUG_SERIAL
  const uint32_t now = millis();
  if (now - lastSensorWarn < DEBUG_SENSOR_WARN_MS) return;
  lastSensorWarn = now;

  Serial.println("[mpu] NOT READY - no gyro data, car keeps auto-spinning");
#endif
}

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
  float y = (yaw + YAW_VIEW_OFFSET) * DEG2RAD;

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

void initNotificationAudio() {
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed!");
    return;
  }

  Serial.println("LittleFS mounted!");

  if (!LittleFS.exists(NOTIFICATION_AUDIO_FILE)) {
    Serial.println("Notification WAV file not found!");
    return;
  }

  i2s_config_t i2sConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = NOTIFICATION_AUDIO_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pinConfig = {
    .bck_io_num = NOTIFICATION_AUDIO_BCLK,
    .ws_io_num = NOTIFICATION_AUDIO_LRC,
    .data_out_num = NOTIFICATION_AUDIO_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  esp_err_t result = i2s_driver_install(I2S_NUM_0, &i2sConfig, 0, NULL);

  if (result != ESP_OK) {
    Serial.printf("I2S install failed: %d\n", result);
    return;
  }

  i2s_set_pin(I2S_NUM_0, &pinConfig);
  i2s_zero_dma_buffer(I2S_NUM_0);

  notificationAudioReady = true;
  Serial.println("Notification audio ready!");
}

void playNotificationAudio() {
  if (!notificationAudioReady || notificationAudioPlaying) return;

  notificationAudioPlaying = true;

  File file = LittleFS.open(NOTIFICATION_AUDIO_FILE, "r");
  if (!file) {
    notificationAudioPlaying = false;
    return;
  }

  // Skip standard WAV header (44 bytes)
  file.seek(44);

  uint8_t buffer[512];
  size_t bytesWritten;

  while (file.available()) {
    int bytesRead = file.read(buffer, sizeof(buffer));

    i2s_write(
      I2S_NUM_0,
      buffer,
      bytesRead,
      &bytesWritten,
      portMAX_DELAY
    );
  }

  file.close();
  notificationAudioPlaying = false;
}

void playLightsAudio() {
  if (!notificationAudioReady || notificationAudioPlaying) return;

  notificationAudioPlaying = true;

  File file = LittleFS.open(LIGHTS_AUDIO_FILE, "r");
  if (!file) {
    notificationAudioPlaying = false;
    return;
  }

  // Skip standard WAV header (44 bytes)
  file.seek(44);

  uint8_t buffer[512];
  size_t bytesWritten;

  while (file.available()) {
    int bytesRead = file.read(buffer, sizeof(buffer));

    i2s_write(
      I2S_NUM_0,
      buffer,
      bytesRead,
      &bytesWritten,
      portMAX_DELAY
    );
  }

  file.close();
  notificationAudioPlaying = false;
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
    playNotificationAudio();
    return;
  }

  if (topicName == topicLedASet) {
    ledAOn = message.equalsIgnoreCase("ON");
    digitalWrite(LED_A_PIN, ledAOn ? HIGH : LOW);
    playLightsAudio();
    publishLedStates();
    return;
  }

  if (topicName == topicLedBSet) {
    ledBOn = message.equalsIgnoreCase("ON");
    digitalWrite(LED_B_PIN, ledBOn ? HIGH : LOW);
    playLightsAudio();
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

uint32_t lastSensorRetry = 0;
const uint32_t SENSOR_RETRY_MS = 5000;

// Forget every motion related value. Called at boot and whenever the sensor is
// (re)initialised, so the animation always restarts from a known state.
void resetMotionState() {
  sensorMoving = false;
  lastMotionTime = millis();
  spinOffset = 0.0f;
  spinBlend = 0.0f;
  spinSpeed = 0.0f;
  yawAngle = 0.0f;
  yawRate = 0.0f;
}

void initSensors() {
  Wire.begin(SDA_PIN, SCL_PIN);

  const byte status = mpu.begin();

#if DEBUG_SERIAL
  Serial.printf("[mpu] begin() status=%u (0 = OK)\n", (unsigned)status);
#endif

  if (status != 0) {
    Serial.println("[mpu] connection failed - check SDA=32, SCL=33, 3V3 and GND");
    drawStatusScreen("MPU6050 ERROR", "CHECK WIRING", TFT_RED);
    sensorReady = false;
    lastSensorRetry = millis();
    return;
  }

  drawStatusScreen("CALIBRATING", "KEEP LEVEL", TFT_CYAN);
  delay(700);

  mpu.calcOffsets();
  mpu.update();

#if DEBUG_SERIAL
  Serial.printf(
    "[mpu] offsets gyro=(%.3f,%.3f,%.3f) deg/s acc=(%.4f,%.4f,%.4f) g\n",
    (double)mpu.getGyroXoffset(),
    (double)mpu.getGyroYoffset(),
    (double)mpu.getGyroZoffset(),
    (double)mpu.getAccXoffset(),
    (double)mpu.getAccYoffset(),
    (double)mpu.getAccZoffset()
  );
#endif

  currentRoll = targetRoll = previousRoll = mpu.getAngleX();
  currentPitch = targetPitch = previousPitch = mpu.getAngleY();

  // Yaw is integrated here (see updateOrientation). The library's angleZ is
  // never wrapped and keeps growing with the gyro bias, which is what made the
  // car creep round for ever even when the board was not moving.
  currentYaw = 0.0f;
  yawAngle = 0.0f;
  yawRate = 0.0f;

  resetMotionState();

  sensorReady = true;

  Serial.println("[mpu] ready - gyro follow armed");
}

void retrySensorIfNeeded() {
  if (millis() - lastSensorRetry < SENSOR_RETRY_MS) return;

  lastSensorRetry = millis();
  Serial.println("[mpu] retrying sensor init");
  initSensors();
}

void updateSensors() {
  if (!sensorReady) {
    debugWarnSensor();
    retrySensorIfNeeded();
    return;
  }

  mpu.update();

  gyroX = mpu.getGyroX();
  gyroY = mpu.getGyroY();
  gyroZ = mpu.getGyroZ();
  gyroMagnitude = sqrtf(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);

  accX = mpu.getAccX();
  accY = mpu.getAccY();
  accZ = mpu.getAccZ();

  targetRoll = mpu.getAngleX();
  targetPitch = mpu.getAngleY();

  // Latch "moving" the moment the gyro wakes up. The hysteresis (enter > exit)
  // keeps the car in gyro-follow mode until the readings are really quiet, so
  // the animation cannot flicker between following and auto-spinning.
  const float motionThreshold =
    sensorMoving ? MOTION_EXIT_THRESHOLD : MOTION_ENTER_THRESHOLD;

  if (gyroMagnitude > motionThreshold) {
    lastMotionTime = millis();
    sensorMoving = true;
  }
}

void updateOrientation(float dt) {
  const uint32_t now = millis();

  // ---- movement detection ---------------------------------------------
  // The gyro is the primary signal, but a real tilt change counts too, so a
  // slow steady rotation still takes the car out of the auto-spin.
  const float tiltDelta =
    fabsf(targetRoll - previousRoll) + fabsf(targetPitch - previousPitch);

  previousRoll = targetRoll;
  previousPitch = targetPitch;

  if (tiltDelta > MOTION_ANGLE_THRESHOLD) {
    lastMotionTime = now;
    sensorMoving = true;
  }

  if (sensorMoving && (now - lastMotionTime) > IDLE_DELAY_MS) {
    sensorMoving = false;
  }

  // ---- yaw ------------------------------------------------------------
  // Integrate the gyro only while the board is moving. The residual Z bias
  // would otherwise keep the car turning for ever. As soon as it goes still the
  // yaw unwinds gently and the auto-spin takes over again.
  const float gyroZClean = (fabsf(gyroZ) < YAW_GYRO_DEADBAND) ? 0.0f : gyroZ;

  yawRate += (gyroZClean - yawRate) * (1.0f - expf(-dt / YAW_RATE_SMOOTHING_TIME));

  if (sensorMoving) {
    yawAngle += yawRate * dt;
  } else {
    yawAngle -= yawAngle * (1.0f - expf(-dt / YAW_RETURN_TIME));
    yawRate = 0.0f;
  }

  // Keep the angle bounded, but shift the smoothed value by the same whole
  // number of turns so wrapping never produces a visual jump.
  if (yawAngle > 180.0f || yawAngle < -180.0f) {
    const float turns = floorf((yawAngle + 180.0f) / 360.0f);
    yawAngle -= turns * 360.0f;
    currentYaw -= turns * 360.0f;
  }

  // ---- smoothing ------------------------------------------------------
  currentRoll += (targetRoll - currentRoll) * SMOOTHING;
  currentPitch += (targetPitch - currentPitch) * SMOOTHING;
  currentYaw += (yawAngle - currentYaw) * SMOOTHING;

  // ---- auto-spin ------------------------------------------------------
  // Fades out quickly when movement is detected and eases back in after
  // IDLE_DELAY_MS of stillness. exp() keeps it frame rate independent.
  const float targetBlend = sensorMoving ? 0.0f : 1.0f;
  const float spinTime = sensorMoving ? SPIN_STOP_TIME : SPIN_START_TIME;

  spinBlend += (targetBlend - spinBlend) * (1.0f - expf(-dt / spinTime));

  // Snap the last fraction so the car really sits still while it is moved.
  if (sensorMoving && spinBlend < 0.002f) spinBlend = 0.0f;
  if (!sensorMoving && spinBlend > 0.998f) spinBlend = 1.0f;

  spinSpeed = AUTO_SPIN_SPEED * spinBlend;
  spinOffset += spinSpeed * dt;

  if (spinOffset >= 360.0f) spinOffset -= 360.0f;
  if (spinOffset < 0.0f) spinOffset += 360.0f;

  // ---- state change log -----------------------------------------------
  static bool loggedMoving = false;

  if (sensorMoving != loggedMoving) {
    loggedMoving = sensorMoving;
    debugLogMotion(sensorMoving);
  }
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

  // connect() blocks. With the default 15 s socket timeout an unreachable
  // broker freezes the loop, which also starves the gyro sampling (the idle
  // timer then fires and the car starts auto-spinning again).
  mqtt.setSocketTimeout(2);
  mqtt.setKeepAlive(15);
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
  initNotificationAudio();

  initTopics();
  initSensors();

  if (!connectToStoredWifi()) {
    startProvisioningAP();
    return;
  }

  initMqtt();

  debugPrintConfig();
  resetMotionState();

  lastFrame = millis();

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

  if (dt > 0.0f) {
    const float instantFps = 1.0f / dt;
    measuredFps = (measuredFps <= 0.0f)
      ? instantFps
      : (measuredFps * 0.9f + instantFps * 0.1f);
  }

  updateOrientation(dt);
  updateFloatingText(dt);
  renderFrame();

  debugPrintState();
}
